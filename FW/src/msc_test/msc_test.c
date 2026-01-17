#include "msc_test/msc_test.h"

#include "app_usbx_host.h"   /* keeps consistency with your project; also provides hhcd_USB_DRD_FS */

#include "ux_api.h"
#include "ux_host_stack.h"

#include "tx_api.h"

#include <stdio.h>
#include <string.h>

/*
 * USB FDD (TEAC) = Mass Storage / UFI / CBI (Control/Bulk/Interrupt)
 *
 * We do NOT use USBX Host Storage class nor FileX.
 * Instead we implement the CBI transport directly using USBX host stack APIs:
 *   1) ADSC (class-specific control transfer) with a 12-byte UFI command block
 *   2) Bulk DATA phase (optional)
 *   3) Interrupt status phase (2 bytes)
 *
 * Endpoints (from your capture):
 *   Bulk OUT : 0x01
 *   Bulk IN  : 0x82
 *   Int  IN  : 0x83 (2 bytes)
 */

/* -------- Wait/Notify (ThreadX) -------- */

static TX_SEMAPHORE g_msc_sem;
static volatile UINT g_sem_inited = 0;

/* Last device pointer notified from ux_host_event_callback(). */
static UX_DEVICE * volatile g_dev = UX_NULL;

void msc_test_rtos_init(void)
{
    if (g_sem_inited) {
        return;
    }
    /* Best effort; if create fails, we keep g_sem_inited=0 and wait() will poll. */
    if (tx_semaphore_create(&g_msc_sem, (CHAR *)"msc_test_sem", 0) == TX_SUCCESS) {
        g_sem_inited = 1;
    }
}

void msc_test_notify(void *dev)
{
    g_dev = (UX_DEVICE *)dev;

    if (g_sem_inited) {
        (void)tx_semaphore_put(&g_msc_sem);
    }
}

void msc_test_wait(void)
{
    if (!g_sem_inited) {
        /* Fallback: polling (shouldn't happen in normal ThreadX setups) */
        while (g_dev == UX_NULL) {
            tx_thread_sleep(1);
        }
        return;
    }

    (void)tx_semaphore_get(&g_msc_sem, TX_WAIT_FOREVER);
}

/* -------- Utilities -------- */

static void dump_hex(const char *title, const UCHAR *buf, ULONG len)
{
    if (title) {
        printf("%s (len=%lu)\r\n", title, (unsigned long)len);
    }
    for (ULONG i = 0; i < len; i++) {
        if ((i % 16u) == 0u) {
            printf("%08lu: ", (unsigned long)i);
        }
        printf("%02X ", (unsigned)buf[i]);
        if ((i % 16u) == 15u) {
            printf("\r\n");
        }
    }
    if ((len % 16u) != 0u) {
        printf("\r\n");
    }
}

static const char *ux_status_str(UINT s)
{
    switch (s) {
    case UX_SUCCESS: return "UX_SUCCESS";
    case UX_ERROR: return "UX_ERROR";
    case UX_NO_CLASS_MATCH: return "UX_NO_CLASS_MATCH";
    case UX_HOST_CLASS_INSTANCE_UNKNOWN: return "UX_HOST_CLASS_INSTANCE_UNKNOWN";
    default: return "UX_*(other)";
    }
}

/* Find endpoints by address within interface 0 alt 0 */
static UINT find_endpoints(UX_CONFIGURATION *cfg,
                           UX_INTERFACE **out_itf,
                           UX_ENDPOINT **out_bulk_out,
                           UX_ENDPOINT **out_bulk_in,
                           UX_ENDPOINT **out_int_in)
{
    *out_itf = UX_NULL;
    *out_bulk_out = UX_NULL;
    *out_bulk_in = UX_NULL;
    *out_int_in = UX_NULL;

    UX_INTERFACE *itf = UX_NULL;
    UINT st = ux_host_stack_configuration_interface_get(cfg, 0 /* interface_index */,
                                                        0 /* alternate_setting_index */, &itf);
    if (st != UX_SUCCESS || itf == UX_NULL) {
        return st;
    }

    /* There are 3 endpoints (excluding EP0). USBX uses endpoint_index 0..N-1. */
    for (UINT i = 0; i < 8; i++) {
        UX_ENDPOINT *ep = UX_NULL;
        st = ux_host_stack_interface_endpoint_get(itf, i, &ep);
        if (st != UX_SUCCESS || ep == UX_NULL) {
            /* Stop when out of range */
            break;
        }

        UCHAR addr = ep->ux_endpoint_descriptor.bEndpointAddress;

        if (addr == 0x01) {
            *out_bulk_out = ep;
        } else if (addr == 0x82) {
            *out_bulk_in = ep;
        } else if (addr == 0x83) {
            *out_int_in = ep;
        }
    }

    *out_itf = itf;

    if (*out_bulk_out && *out_bulk_in && *out_int_in) {
        return UX_SUCCESS;
    }

    return UX_ERROR;
}

/* Submit a transfer on a given endpoint. */
static UINT endpoint_xfer(UX_ENDPOINT *ep, UCHAR *buf, ULONG len, ULONG timeout_ms)
{
    if (ep == UX_NULL) {
        return UX_ERROR;
    }

    UX_TRANSFER *t = &ep->ux_endpoint_transfer_request;
    t->ux_transfer_request_endpoint = ep;
    t->ux_transfer_request_data_pointer = buf;
    t->ux_transfer_request_requested_length = len;
    t->ux_transfer_request_timeout_value = timeout_ms;

    UINT st = ux_host_stack_transfer_request(t);
    if (st != UX_SUCCESS) {
        return st;
    }
    return t->ux_transfer_request_completion_code;
}

/* CBI: ADSC control transfer on EP0 with UFI 12-byte command block. */
static UINT cbi_adsc(UX_DEVICE *dev, UINT interface_number,
                     UCHAR cmdblk[12], ULONG timeout_ms)
{
    if (dev == UX_NULL) {
        return UX_ERROR;
    }

    UX_ENDPOINT *ep0 = &dev->ux_device_control_endpoint;
    UX_TRANSFER *t = &ep0->ux_endpoint_transfer_request;

    t->ux_transfer_request_endpoint = ep0;

    /* bmRequestType: 0x21 = Host-to-Device | Class | Interface */
    t->ux_transfer_request_type = 0x21;
    /* bRequest: 0x00 = ADSC (per CBI) */
    t->ux_transfer_request_function = 0x00;
    /* wValue */
    t->ux_transfer_request_value = 0;
    /* wIndex = interface number */
    t->ux_transfer_request_index = interface_number;

    t->ux_transfer_request_data_pointer = cmdblk;
    t->ux_transfer_request_requested_length = 12;
    t->ux_transfer_request_timeout_value = timeout_ms;

    UINT st = ux_host_stack_transfer_request(t);
    if (st != UX_SUCCESS) {
        return st;
    }
    return t->ux_transfer_request_completion_code;
}

/* Read 2-byte interrupt status (CBI). */
static UINT cbi_status(UX_ENDPOINT *ep_int_in, UCHAR status2[2], ULONG timeout_ms)
{
    memset(status2, 0, 2);
    return endpoint_xfer(ep_int_in, status2, 2, timeout_ms);
}

/* Execute one CBI(UFI) command: ADSC -> optional Bulk data -> Interrupt status. */
static UINT cbi_cmd(UX_DEVICE *dev,
                    UINT interface_number,
                    UX_ENDPOINT *ep_bulk_out,
                    UX_ENDPOINT *ep_bulk_in,
                    UX_ENDPOINT *ep_int_in,
                    const UCHAR cmdblk_in[12],
                    UCHAR *data,
                    ULONG data_len,
                    int data_in /* 1: bulk IN, 0: bulk OUT */)
{
    UCHAR cmdblk[12];
    memcpy(cmdblk, cmdblk_in, 12);

    UINT st = cbi_adsc(dev, interface_number, cmdblk, 5000);
    if (st != UX_SUCCESS) {
        printf("[msc_test] ADSC failed: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
        return st;
    }

    if (data_len != 0 && data != UX_NULL) {
        if (data_in) {
            st = endpoint_xfer(ep_bulk_in, data, data_len, 5000);
        } else {
            st = endpoint_xfer(ep_bulk_out, data, data_len, 5000);
        }
        if (st != UX_SUCCESS) {
            printf("[msc_test] Bulk %s failed: %s (%u)\r\n",
                   data_in ? "IN" : "OUT", ux_status_str(st), (unsigned)st);
            return st;
        }
    }

    UCHAR st2[2];
    st = cbi_status(ep_int_in, st2, 5000);
    if (st != UX_SUCCESS) {
        printf("[msc_test] Interrupt status failed: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
        return st;
    }

    /* For many CBI devices, status2[0] holds a status code. Keep it for debug. */
    if (st2[0] != 0x00 || st2[1] != 0x00) {
        printf("[msc_test] CBI status = %02X %02X\r\n", st2[0], st2[1]);
    }

    return UX_SUCCESS;
}

/* Build 12-byte UFI command block from a 6-byte SCSI CDB. */
static void ufi_from_cdb6(UCHAR out12[12],
                          UCHAR op, UCHAR b1, UCHAR b2, UCHAR b3, UCHAR b4, UCHAR b5)
{
    memset(out12, 0, 12);
    out12[0] = op;
    out12[1] = b1;
    out12[2] = b2;
    out12[3] = b3;
    out12[4] = b4;
    out12[5] = b5;
}

/* Build 12-byte UFI command block from a 10-byte SCSI CDB. */
static void ufi_from_cdb10(UCHAR out12[12],
                           const UCHAR cdb10[10])
{
    memset(out12, 0, 12);
    memcpy(out12, cdb10, 10);
}

/* -------- Main test -------- */

#ifndef MSC_TEST_READ_BLOCK_SIZE
#define MSC_TEST_READ_BLOCK_SIZE 512u
#endif

void msc_test(void)
{
    UX_DEVICE *dev = g_dev;
    if (dev == UX_NULL) {
        printf("[msc_test] no device\r\n");
        return;
    }

    printf("\r\n[msc_test] start (CBI/UFI)\r\n");

    /* Select configuration 1 (index 0 -> the only configuration). */
    UX_CONFIGURATION *cfg = UX_NULL;
    UINT st = ux_host_stack_device_configuration_get(dev, 0, &cfg);
    if (st != UX_SUCCESS || cfg == UX_NULL) {
        printf("[msc_test] configuration_get failed: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
        return;
    }

    st = ux_host_stack_device_configuration_select(cfg);
    if (st != UX_SUCCESS) {
        printf("[msc_test] configuration_select failed: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
        return;
    }

    UX_INTERFACE *itf = UX_NULL;
    UX_ENDPOINT *ep_bulk_out = UX_NULL;
    UX_ENDPOINT *ep_bulk_in = UX_NULL;
    UX_ENDPOINT *ep_int_in = UX_NULL;

    st = find_endpoints(cfg, &itf, &ep_bulk_out, &ep_bulk_in, &ep_int_in);
    if (st != UX_SUCCESS) {
        printf("[msc_test] endpoint discovery failed\r\n");
        return;
    }

    /* Interface number is 0 for your device/interface #0. */
    const UINT interface_number = 0;

    /* 0x03 REQUEST SENSE (alloc length 18) */
    UCHAR rs_buf[18];
    memset(rs_buf, 0, sizeof(rs_buf));
    UCHAR ufi[12];
    ufi_from_cdb6(ufi, 0x03, 0x00, 0x00, 0x00, (UCHAR)sizeof(rs_buf), 0x00);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, rs_buf, (ULONG)sizeof(rs_buf), 1);
    printf("[msc_test] REQUEST SENSE: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) dump_hex("REQUEST SENSE data", rs_buf, (ULONG)sizeof(rs_buf));

    /* 0x12 INQUIRY (alloc length 36) */
    UCHAR inq_buf[36];
    memset(inq_buf, 0, sizeof(inq_buf));
    ufi_from_cdb6(ufi, 0x12, 0x00, 0x00, 0x00, (UCHAR)sizeof(inq_buf), 0x00);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, inq_buf, (ULONG)sizeof(inq_buf), 1);
    printf("[msc_test] INQUIRY: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) dump_hex("INQUIRY data", inq_buf, (ULONG)sizeof(inq_buf));

    /* 0x00 TEST UNIT READY (no data) */
    ufi_from_cdb6(ufi, 0x00, 0, 0, 0, 0, 0);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, UX_NULL, 0, 1);
    printf("[msc_test] TEST UNIT READY: %s (%u)\r\n", ux_status_str(st), (unsigned)st);

    /* 0x1E PREVENT / ALLOW MEDIUM REMOVAL (prevent=1, no data) */
    ufi_from_cdb6(ufi, 0x1E, 0x00, 0x00, 0x00, 0x01, 0x00);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, UX_NULL, 0, 1);
    printf("[msc_test] PREVENT/ALLOW (prevent=1): %s (%u)\r\n", ux_status_str(st), (unsigned)st);

    /* 0x03 REQUEST SENSE again */
    memset(rs_buf, 0, sizeof(rs_buf));
    ufi_from_cdb6(ufi, 0x03, 0x00, 0x00, 0x00, (UCHAR)sizeof(rs_buf), 0x00);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, rs_buf, (ULONG)sizeof(rs_buf), 1);
    printf("[msc_test] REQUEST SENSE(2): %s (%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) dump_hex("REQUEST SENSE(2) data", rs_buf, (ULONG)sizeof(rs_buf));

    /* 0x25 READ CAPACITY(10) (8 bytes) */
    UCHAR rc_buf[8];
    memset(rc_buf, 0, sizeof(rc_buf));
    const UCHAR cdb_rc10[10] = { 0x25, 0,0,0,0,0, 0,0,0,0 };
    ufi_from_cdb10(ufi, cdb_rc10);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, rc_buf, (ULONG)sizeof(rc_buf), 1);
    printf("[msc_test] READ CAPACITY(10): %s (%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) dump_hex("READ CAPACITY(10) data", rc_buf, (ULONG)sizeof(rc_buf));

    /* 0x5A MODE SENSE(10) - PageCode=0x3F(all), alloc length=8 */
    UCHAR ms_buf[8];
    memset(ms_buf, 0, sizeof(ms_buf));
    UCHAR cdb_ms10[10] = {
        0x5A, 0x00,
        0x3F, 0x00,    /* PC=0, PageCode=0x3F (all pages) */
        0x00, 0x00,
        0x00, 0x00,    /* allocation length (BE) */
        0x00
    };
    /* allocation length = 8 */
    cdb_ms10[7] = 0x00;
    cdb_ms10[8] = (UCHAR)sizeof(ms_buf);

    ufi_from_cdb10(ufi, cdb_ms10);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, ms_buf, (ULONG)sizeof(ms_buf), 1);
    printf("[msc_test] MODE SENSE(10) alloc=8: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) dump_hex("MODE SENSE(10) data", ms_buf, (ULONG)sizeof(ms_buf));

    /* 0x12 INQUIRY again */
    memset(inq_buf, 0, sizeof(inq_buf));
    ufi_from_cdb6(ufi, 0x12, 0x00, 0x00, 0x00, (UCHAR)sizeof(inq_buf), 0x00);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, inq_buf, (ULONG)sizeof(inq_buf), 1);
    printf("[msc_test] INQUIRY(2): %s (%u)\r\n", ux_status_str(st), (unsigned)st);

    /* 0x28 READ(10) LBA=0, Transfer Length=1 block */
    static UCHAR lba0_buf[MSC_TEST_READ_BLOCK_SIZE];
    memset(lba0_buf, 0, sizeof(lba0_buf));

    UCHAR cdb_rd10[10] = {
        0x28, 0x00,
        0x00, 0x00, 0x00, 0x00,   /* LBA = 0 */
        0x00,
        0x00, 0x01,               /* Transfer Length = 1 */
        0x00
    };
    ufi_from_cdb10(ufi, cdb_rd10);

    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, lba0_buf, (ULONG)sizeof(lba0_buf), 1);

    printf("[msc_test] READ(10) LBA0 len=1block: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) {
        dump_hex("LBA0 dump", lba0_buf, (ULONG)sizeof(lba0_buf));
    }

    printf("[msc_test] done\r\n");
}

void msc_test_thread_entry(ULONG argument)
{
    (void)argument;

    while (1) {
        msc_test_wait();
        msc_test();
    }
}
