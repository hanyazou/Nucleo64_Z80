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
 *
 * IMPORTANT:
 *   If CubeMX "USBX Host Class Storage (MSC)" is enabled, USBX may try to bind
 *   the storage class driver and fail because this device is CBI/UFI (not BOT).
 *   That often triggers UX_HOST_CLASS_PROTOCOL_ERROR / UX_DEVICE_ENUMERATION_FAILURE,
 *   leaving endpoints unactivated (UX_ENDPOINT_HANDLE_UNKNOWN).
 *
 *   Recommended: disable USBX host storage class in CubeMX (keep Host stack only),
 *   or comment out the storage class register in app_usbx_host.c.
 *
 * This file tries to (re-)select configuration and activate interface/endpoints
 * before issuing transfers, but it cannot recover if enumeration is aborted by
 * a class driver before endpoints exist.
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
#ifdef UX_ENDPOINT_HANDLE_UNKNOWN
    case UX_ENDPOINT_HANDLE_UNKNOWN: return "UX_ENDPOINT_HANDLE_UNKNOWN";
#endif
    default: return "UX_*(other)";
    }
}

static void print_xfer_result(const char *tag, UX_TRANSFER *t, UINT call_status)
{
    printf("[msc_test] %s: call=%s(%u) cc=%s(%u) actual=%lu req=%lu\r\n",
           tag,
           ux_status_str(call_status), (unsigned)call_status,
           ux_status_str(t->ux_transfer_request_completion_code),
           (unsigned)t->ux_transfer_request_completion_code,
           (unsigned long)t->ux_transfer_request_actual_length,
           (unsigned long)t->ux_transfer_request_requested_length);
}

/* -------- Endpoint discovery / activation -------- */

static UINT activate_config_and_interface(UX_DEVICE *dev,
                                         UX_CONFIGURATION **out_cfg,
                                         UX_INTERFACE **out_itf)
{
    *out_cfg = UX_NULL;
    *out_itf = UX_NULL;

    UX_CONFIGURATION *cfg = UX_NULL;
    UINT st = ux_host_stack_device_configuration_get(dev, 0, &cfg);
    if (st != UX_SUCCESS || cfg == UX_NULL) {
        return st;
    }

    /* Select configuration (sends SET_CONFIGURATION). */
    st = ux_host_stack_device_configuration_select(cfg);
    if (st != UX_SUCCESS) {
        return st;
    }

    /* Some ports require explicit activation; safe to call. */
    (void)ux_host_stack_device_configuration_activate(cfg);

    /* Interface #0, alternate #0. */
    UX_INTERFACE *itf = UX_NULL;
    st = ux_host_stack_configuration_interface_get(cfg, 0, 0, &itf);
    if (st != UX_SUCCESS || itf == UX_NULL) {
        return st;
    }

    /* Select interface setting (activates endpoints for that alternate). */
    st = ux_host_stack_interface_setting_select(itf);
    if (st != UX_SUCCESS) {
        return st;
    }

    *out_cfg = cfg;
    *out_itf = itf;
    return UX_SUCCESS;
}

static UINT find_ep_by_addr(UX_INTERFACE *itf, UCHAR addr, UX_ENDPOINT **out_ep)
{
    *out_ep = UX_NULL;

    for (UINT i = 0; i < 8; i++) {
        UX_ENDPOINT *ep = UX_NULL;
        UINT st = ux_host_stack_interface_endpoint_get(itf, i, &ep);
        if (st != UX_SUCCESS || ep == UX_NULL) {
            break;
        }
        if (ep->ux_endpoint_descriptor.bEndpointAddress == addr) {
            *out_ep = ep;
            return UX_SUCCESS;
        }
    }

    return UX_ERROR;
}

static UINT get_cbi_endpoints(UX_INTERFACE *itf,
                             UX_ENDPOINT **out_bulk_out,
                             UX_ENDPOINT **out_bulk_in,
                             UX_ENDPOINT **out_int_in)
{
    *out_bulk_out = UX_NULL;
    *out_bulk_in = UX_NULL;
    *out_int_in = UX_NULL;

    (void)find_ep_by_addr(itf, 0x01, out_bulk_out);
    (void)find_ep_by_addr(itf, 0x82, out_bulk_in);
    (void)find_ep_by_addr(itf, 0x83, out_int_in);

    if (*out_bulk_out && *out_bulk_in && *out_int_in) {
        return UX_SUCCESS;
    }
    return UX_ERROR;
}

/* -------- Transfers -------- */

static UINT endpoint_xfer(UX_ENDPOINT *ep, UCHAR *buf, ULONG len, ULONG timeout_ms, const char *tag)
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
    print_xfer_result(tag, t, st);

    if (st != UX_SUCCESS) {
        return st;
    }
    return t->ux_transfer_request_completion_code;
}

static UINT cbi_adsc(UX_DEVICE *dev, UINT interface_number,
                     UCHAR cmdblk[12], ULONG timeout_ms)
{
    UX_ENDPOINT *ep0 = &dev->ux_device_control_endpoint;
    UX_TRANSFER *t = &ep0->ux_endpoint_transfer_request;

    t->ux_transfer_request_endpoint = ep0;

    /* bmRequestType: 0x21 = Host-to-Device | Class | Interface */
    t->ux_transfer_request_type = 0x21;
    /* bRequest: 0x00 = ADSC */
    t->ux_transfer_request_function = 0x00;
    t->ux_transfer_request_value = 0;
    t->ux_transfer_request_index = interface_number;

    t->ux_transfer_request_data_pointer = cmdblk;
    t->ux_transfer_request_requested_length = 12;
    t->ux_transfer_request_timeout_value = timeout_ms;

    UINT st = ux_host_stack_transfer_request(t);
    print_xfer_result("ADSC(EP0)", t, st);

    if (st != UX_SUCCESS) {
        return st;
    }
    return t->ux_transfer_request_completion_code;
}

static UINT cbi_status(UX_ENDPOINT *ep_int_in, UCHAR status2[2], ULONG timeout_ms)
{
    memset(status2, 0, 2);
    return endpoint_xfer(ep_int_in, status2, 2, timeout_ms, "INT-IN status");
}

static UINT cbi_cmd(UX_DEVICE *dev,
                    UINT interface_number,
                    UX_ENDPOINT *ep_bulk_out,
                    UX_ENDPOINT *ep_bulk_in,
                    UX_ENDPOINT *ep_int_in,
                    const UCHAR cmdblk_in[12],
                    UCHAR *data,
                    ULONG data_len,
                    int data_in)
{
    UCHAR cmdblk[12];
    memcpy(cmdblk, cmdblk_in, 12);

    UINT st = cbi_adsc(dev, interface_number, cmdblk, 5000);
    if (st != UX_SUCCESS) {
        return st;
    }

    if (data_len != 0 && data != UX_NULL) {
        if (data_in) {
            st = endpoint_xfer(ep_bulk_in, data, data_len, 5000, "Bulk IN");
        } else {
            st = endpoint_xfer(ep_bulk_out, data, data_len, 5000, "Bulk OUT");
        }
        if (st != UX_SUCCESS) {
            return st;
        }
    }

    UCHAR st2[2];
    st = cbi_status(ep_int_in, st2, 5000);
    if (st != UX_SUCCESS) {
        return st;
    }

    if (st2[0] != 0x00 || st2[1] != 0x00) {
        printf("[msc_test] CBI status bytes = %02X %02X\r\n", st2[0], st2[1]);
    }

    return UX_SUCCESS;
}

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

static void ufi_from_cdb10(UCHAR out12[12], const UCHAR cdb10[10])
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

    UX_CONFIGURATION *cfg = UX_NULL;
    UX_INTERFACE *itf = UX_NULL;

    UINT st = activate_config_and_interface(dev, &cfg, &itf);
    if (st != UX_SUCCESS) {
        printf("[msc_test] activate_config_and_interface failed: %s (%u)\r\n",
               ux_status_str(st), (unsigned)st);
        return;
    }

    UX_ENDPOINT *ep_bulk_out = UX_NULL;
    UX_ENDPOINT *ep_bulk_in = UX_NULL;
    UX_ENDPOINT *ep_int_in = UX_NULL;

    st = get_cbi_endpoints(itf, &ep_bulk_out, &ep_bulk_in, &ep_int_in);
    if (st != UX_SUCCESS) {
        printf("[msc_test] endpoint discovery failed (need 0x01/0x82/0x83)\r\n");
        return;
    }

    const UINT interface_number = 0;

    UCHAR ufi[12];

    UCHAR rs_buf[18];
    memset(rs_buf, 0, sizeof(rs_buf));
    ufi_from_cdb6(ufi, 0x03, 0x00, 0x00, 0x00, (UCHAR)sizeof(rs_buf), 0x00);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, rs_buf, (ULONG)sizeof(rs_buf), 1);
    printf("[msc_test] REQUEST SENSE: %s (%u)\r\n", ux_status_str(st), (unsigned)st);

    UCHAR inq_buf[36];
    memset(inq_buf, 0, sizeof(inq_buf));
    ufi_from_cdb6(ufi, 0x12, 0x00, 0x00, 0x00, (UCHAR)sizeof(inq_buf), 0x00);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, inq_buf, (ULONG)sizeof(inq_buf), 1);
    printf("[msc_test] INQUIRY: %s (%u)\r\n", ux_status_str(st), (unsigned)st);

    ufi_from_cdb6(ufi, 0x00, 0, 0, 0, 0, 0);
    st = cbi_cmd(dev, interface_number, ep_bulk_out, ep_bulk_in, ep_int_in,
                 ufi, UX_NULL, 0, 1);
    printf("[msc_test] TEST UNIT READY: %s (%u)\r\n", ux_status_str(st), (unsigned)st);

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
    printf("[msc_test] READ(10) LBA0: %s (%u)\r\n", ux_status_str(st), (unsigned)st);
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
