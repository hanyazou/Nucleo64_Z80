#include "msc_test/msc_test.h"

#include "app_usbx_host.h"

#include "ux_api.h"
#include "ux_host_stack.h"
#include "tx_api.h"

#include <stdio.h>
#include <string.h>

/*
 * USB FDD (TEAC) = Mass Storage / UFI / CBI (Control/Bulk/Interrupt)
 *
 * - No USBX Host Storage class / no FileX.
 * - Implement CBI transport directly:
 *     ADSC(Control EP0) -> Bulk data -> Interrupt status(2 bytes)
 *
 * Your endpoints:
 *   Bulk OUT : 0x01
 *   Bulk IN  : 0x82
 *   Int  IN  : 0x83 (2 bytes)
 *
 * Notes from CBI spec:
 * - Data-In: device may NAK Bulk IN until data is ready; host must keep polling.
 * - Interrupt endpoint signals command completion.
 * - Command Block Reset is SEND DIAGNOSTIC: 1Dh 04h FFh FFh FFh FFh ... then ClearFeature(HALT) on bulk pipes.
 */

static TX_SEMAPHORE g_msc_sem;
static volatile UINT g_sem_inited = 0;
static UX_DEVICE * volatile g_dev = UX_NULL;

#ifndef MSC_TEST_TIMEOUT_MS
#define MSC_TEST_TIMEOUT_MS   (30000u)
#endif

#ifndef MSC_TEST_RETRIES
#define MSC_TEST_RETRIES      (3u)
#endif

#ifndef MSC_TEST_READ_BLOCK_SIZE
#define MSC_TEST_READ_BLOCK_SIZE 512u
#endif

void msc_test_rtos_init(void)
{
    if (g_sem_inited) return;
    if (tx_semaphore_create(&g_msc_sem, (CHAR *)"msc_test_sem", 0) == TX_SUCCESS) {
        g_sem_inited = 1;
    }
}

void msc_test_notify(void *dev)
{
    g_dev = (UX_DEVICE *)dev;
    if (g_sem_inited) (void)tx_semaphore_put(&g_msc_sem);
}

void msc_test_wait(void)
{
    if (!g_sem_inited) {
        while (g_dev == UX_NULL) tx_thread_sleep(1);
        return;
    }
    (void)tx_semaphore_get(&g_msc_sem, TX_WAIT_FOREVER);
}

static void dump_hex(const char *title, const UCHAR *buf, ULONG len)
{
    if (title) printf("%s (len=%lu)\r\n", title, (unsigned long)len);
    for (ULONG i = 0; i < len; i++) {
        if ((i % 16u) == 0u) printf("%08lu: ", (unsigned long)i);
        printf("%02X ", (unsigned)buf[i]);
        if ((i % 16u) == 15u) printf("\r\n");
    }
    if ((len % 16u) != 0u) printf("\r\n");
}

static const char *ux_status_str(UINT s)
{
    switch (s) {
    case UX_SUCCESS: return "UX_SUCCESS";
    case UX_ERROR: return "UX_ERROR";
#ifdef UX_TRANSFER_STALLED
    case UX_TRANSFER_STALLED: return "UX_TRANSFER_STALLED";
#endif
#ifdef UX_TRANSFER_TIMEOUT
    case UX_TRANSFER_TIMEOUT: return "UX_TRANSFER_TIMEOUT";
#endif
#ifdef UX_ENDPOINT_HANDLE_UNKNOWN
    case UX_ENDPOINT_HANDLE_UNKNOWN: return "UX_ENDPOINT_HANDLE_UNKNOWN";
#endif
    default: return "UX_*(other)";
    }
}

static void print_xfer(const char *tag, UX_TRANSFER *t, UINT call_status)
{
    printf("[msc_test] %s: call=%s(%u) cc=%s(%u) actual=%lu req=%lu\r\n",
           tag,
           ux_status_str(call_status), (unsigned)call_status,
           ux_status_str(t->ux_transfer_request_completion_code),
           (unsigned)t->ux_transfer_request_completion_code,
           (unsigned long)t->ux_transfer_request_actual_length,
           (unsigned long)t->ux_transfer_request_requested_length);
}

static UINT activate_config_and_interface(UX_DEVICE *dev,
                                         UX_CONFIGURATION **out_cfg,
                                         UX_INTERFACE **out_itf)
{
    *out_cfg = UX_NULL;
    *out_itf = UX_NULL;

    UX_CONFIGURATION *cfg = UX_NULL;
    UINT st = ux_host_stack_device_configuration_get(dev, 0, &cfg);
    if (st != UX_SUCCESS || cfg == UX_NULL) return st;

    st = ux_host_stack_device_configuration_select(cfg);
    if (st != UX_SUCCESS) return st;

    (void)ux_host_stack_device_configuration_activate(cfg);

    UX_INTERFACE *itf = UX_NULL;
    st = ux_host_stack_configuration_interface_get(cfg, 0, 0, &itf);
    if (st != UX_SUCCESS || itf == UX_NULL) return st;

    st = ux_host_stack_interface_setting_select(itf);
    if (st != UX_SUCCESS) return st;

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
        if (st != UX_SUCCESS || ep == UX_NULL) break;
        if (ep->ux_endpoint_descriptor.bEndpointAddress == addr) {
            *out_ep = ep;
            return UX_SUCCESS;
        }
    }
    return UX_ERROR;
}

static UINT get_cbi_eps(UX_INTERFACE *itf, UX_ENDPOINT **ep_out, UX_ENDPOINT **ep_in, UX_ENDPOINT **ep_int)
{
    *ep_out = *ep_in = *ep_int = UX_NULL;
    (void)find_ep_by_addr(itf, 0x01, ep_out);
    (void)find_ep_by_addr(itf, 0x82, ep_in);
    (void)find_ep_by_addr(itf, 0x83, ep_int);
    return (*ep_out && *ep_in && *ep_int) ? UX_SUCCESS : UX_ERROR;
}

static UINT endpoint_xfer(UX_ENDPOINT *ep, UCHAR *buf, ULONG len, ULONG timeout_ms, const char *tag)
{
    UX_TRANSFER *t = &ep->ux_endpoint_transfer_request;

    /* Clear setup-related fields for non-control. */
    t->ux_transfer_request_type = 0;
    t->ux_transfer_request_function = 0;
    t->ux_transfer_request_value = 0;
    t->ux_transfer_request_index = 0;

    t->ux_transfer_request_endpoint = ep;
    t->ux_transfer_request_data_pointer = buf;
    t->ux_transfer_request_requested_length = len;
    t->ux_transfer_request_timeout_value = timeout_ms;

    t->ux_transfer_request_maximum_length = len;
    t->ux_transfer_request_packet_length  = ep->ux_endpoint_descriptor.wMaxPacketSize;

    UINT st = ux_host_stack_transfer_request(t);
    print_xfer(tag, t, st);
    if (st != UX_SUCCESS) return st;
    return t->ux_transfer_request_completion_code;
}

static UINT clear_halt(UX_DEVICE *dev, UCHAR ep_addr)
{
    UX_ENDPOINT *ep0 = &dev->ux_device_control_endpoint;
    UX_TRANSFER *t = &ep0->ux_endpoint_transfer_request;

    t->ux_transfer_request_endpoint = ep0;

    /* 0x02 = Host->Dev | Standard | Endpoint */
    t->ux_transfer_request_type = 0x02;
    /* CLEAR_FEATURE = 0x01 */
    t->ux_transfer_request_function = 0x01;
    /* ENDPOINT_HALT = 0 */
    t->ux_transfer_request_value = 0x0000;
    t->ux_transfer_request_index = ep_addr;

    t->ux_transfer_request_data_pointer = UX_NULL;
    t->ux_transfer_request_requested_length = 0;
    t->ux_transfer_request_timeout_value = MSC_TEST_TIMEOUT_MS;

    UINT st = ux_host_stack_transfer_request(t);
    print_xfer("CLEAR_FEATURE(HALT)", t, st);
    if (st != UX_SUCCESS) return st;
    return t->ux_transfer_request_completion_code;
}

static UINT cbi_adsc(UX_DEVICE *dev, UINT ifnum, UCHAR cmdblk[12])
{
    UX_ENDPOINT *ep0 = &dev->ux_device_control_endpoint;
    UX_TRANSFER *t = &ep0->ux_endpoint_transfer_request;

    t->ux_transfer_request_endpoint = ep0;

    /* 0x21 = Host->Dev | Class | Interface */
    t->ux_transfer_request_type = 0x21;
    /* ADSC = 0x00 */
    t->ux_transfer_request_function = 0x00;
    t->ux_transfer_request_value = 0;
    t->ux_transfer_request_index = ifnum;

    t->ux_transfer_request_data_pointer = cmdblk;
    t->ux_transfer_request_requested_length = 12;
    t->ux_transfer_request_timeout_value = MSC_TEST_TIMEOUT_MS;

    t->ux_transfer_request_maximum_length = 12;
    t->ux_transfer_request_packet_length  = ep0->ux_endpoint_descriptor.wMaxPacketSize;

    UINT st = ux_host_stack_transfer_request(t);
    print_xfer("ADSC(EP0)", t, st);
    if (st != UX_SUCCESS) return st;
    return t->ux_transfer_request_completion_code;
}

static UINT cbi_int_status(UX_ENDPOINT *ep_int, UCHAR st2[2])
{
    memset(st2, 0, 2);
    return endpoint_xfer(ep_int, st2, 2, MSC_TEST_TIMEOUT_MS, "INT-IN status");
}

static void ufi_from_cdb6(UCHAR out12[12], UCHAR op, UCHAR b1, UCHAR b2, UCHAR b3, UCHAR b4, UCHAR b5)
{
    memset(out12, 0, 12);
    out12[0] = op; out12[1] = b1; out12[2] = b2; out12[3] = b3; out12[4] = b4; out12[5] = b5;
}

static void ufi_from_cdb10(UCHAR out12[12], const UCHAR cdb10[10])
{
    memset(out12, 0, 12);
    memcpy(out12, cdb10, 10);
}

static UINT cbi_command_block_reset(UX_DEVICE *dev, UINT ifnum)
{
    UCHAR ufi[12];
    memset(ufi, 0xFF, sizeof(ufi));
    ufi[0] = 0x1D;
    ufi[1] = 0x04;

    printf("[msc_test] >>> Command Block Reset\r\n");
    (void)cbi_adsc(dev, ifnum, ufi);

    (void)clear_halt(dev, 0x01);
    (void)clear_halt(dev, 0x82);

    return UX_SUCCESS;
}

static UINT cbi_exec(UX_DEVICE *dev, UINT ifnum,
                     UX_ENDPOINT *ep_out, UX_ENDPOINT *ep_in, UX_ENDPOINT *ep_int,
                     const UCHAR cmdblk_in[12],
                     UCHAR *data, ULONG data_len, int data_in)
{
    for (UINT attempt = 0; attempt < MSC_TEST_RETRIES; attempt++) {

        UCHAR cmdblk[12];
        memcpy(cmdblk, cmdblk_in, 12);

        UINT st = cbi_adsc(dev, ifnum, cmdblk);
        if (st != UX_SUCCESS) {
            cbi_command_block_reset(dev, ifnum);
            continue;
        }

        if (data && data_len) {
            st = data_in ?
                 endpoint_xfer(ep_in, data, data_len, MSC_TEST_TIMEOUT_MS, "Bulk IN") :
                 endpoint_xfer(ep_out, data, data_len, MSC_TEST_TIMEOUT_MS, "Bulk OUT");
            if (st != UX_SUCCESS) {
                cbi_command_block_reset(dev, ifnum);
                continue;
            }
        }

        UCHAR st2[2];
        st = cbi_int_status(ep_int, st2);
        if (st != UX_SUCCESS) {
            cbi_command_block_reset(dev, ifnum);
            continue;
        }

        if (st2[0] != 0x00 || st2[1] != 0x00) {
            printf("[msc_test] CBI status bytes = %02X %02X\r\n", st2[0], st2[1]);
        }

        return UX_SUCCESS;
    }

    return UX_ERROR;
}

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
        printf("[msc_test] activate_config_and_interface failed: %s(%u)\r\n",
               ux_status_str(st), (unsigned)st);
        return;
    }

    UX_ENDPOINT *ep_out = UX_NULL, *ep_in = UX_NULL, *ep_int = UX_NULL;
    if (get_cbi_eps(itf, &ep_out, &ep_in, &ep_int) != UX_SUCCESS) {
        printf("[msc_test] endpoints not found\r\n");
        return;
    }

    const UINT ifnum = 0;
    UCHAR ufi[12];

    UCHAR rs[18]; memset(rs, 0, sizeof(rs));
    ufi_from_cdb6(ufi, 0x03, 0,0,0, (UCHAR)sizeof(rs), 0);
    st = cbi_exec(dev, ifnum, ep_out, ep_in, ep_int, ufi, rs, sizeof(rs), 1);
    printf("[msc_test] REQUEST SENSE: %s(%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) dump_hex("REQUEST SENSE", rs, sizeof(rs));

    UCHAR inq[36]; memset(inq, 0, sizeof(inq));
    ufi_from_cdb6(ufi, 0x12, 0,0,0, (UCHAR)sizeof(inq), 0);
    st = cbi_exec(dev, ifnum, ep_out, ep_in, ep_int, ufi, inq, sizeof(inq), 1);
    printf("[msc_test] INQUIRY: %s(%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) dump_hex("INQUIRY", inq, sizeof(inq));

    ufi_from_cdb6(ufi, 0x00, 0,0,0,0,0);
    st = cbi_exec(dev, ifnum, ep_out, ep_in, ep_int, ufi, UX_NULL, 0, 1);
    printf("[msc_test] TEST UNIT READY: %s(%u)\r\n", ux_status_str(st), (unsigned)st);

    static UCHAR lba0[MSC_TEST_READ_BLOCK_SIZE];
    memset(lba0, 0, sizeof(lba0));
    UCHAR cdb_rd10[10] = { 0x28,0x00, 0,0,0,0, 0, 0,1, 0 };
    ufi_from_cdb10(ufi, cdb_rd10);
    st = cbi_exec(dev, ifnum, ep_out, ep_in, ep_int, ufi, lba0, sizeof(lba0), 1);
    printf("[msc_test] READ(10) LBA0: %s(%u)\r\n", ux_status_str(st), (unsigned)st);
    if (st == UX_SUCCESS) dump_hex("LBA0", lba0, sizeof(lba0));

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
