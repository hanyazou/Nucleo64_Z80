/*
 * board_test.c
 *
 *  Created on: Mar 25, 2024
 *      Author: hanyazou
 */

#include "main.h"
#include "board_test.h"

#include <stdio.h>
#include <stdint.h>

#define CAT(x, y) _CAT(x, y)
#define _CAT(x, y) x ## y
#define set_pin(pin, v) ((v) ? (CAT(pin,_GPIO_Port)->ODR |= CAT(pin,_Pin)) : \
                         (CAT(pin,_GPIO_Port)->ODR &= ~CAT(pin,_Pin)))
#define get_pin(pin) ((CAT(pin,_GPIO_Port)->IDR & CAT(pin,_Pin)) ? 1 : 0)
#define set_pin_dir(pin, v) \
    ((v) ? \
     (CAT(pin,_GPIO_Port)->MODER &= ~((CAT(pin,_Pin))*(CAT(pin,_Pin))*3)) : \
     (CAT(pin,_GPIO_Port)->MODER |=  ((CAT(pin,_Pin))*(CAT(pin,_Pin))  )))

typedef uint8_t __bit;

static uint8_t data_pins(void) { return (uint8_t)(GPIOB->IDR >> 8); }
static void set_data_pins(uint8_t v) { GPIOB->ODR = ((GPIOB->ODR & 0xff) | ((uint16_t)(v) << 8)); }
static void set_data_dir(uint8_t v) {
    GPIOB->MODER = ((GPIOB->MODER & 0x0000ffff) | ((v) ? 0 : 0x55550000));
}
static uint16_t addr_pins(void) { return (uint16_t)(GPIOC->IDR & 0xffff); }
static void set_addr_pins(uint16_t v) { GPIOC->ODR = (v); }
static void set_addr_dir(uint8_t v) { GPIOC->MODER = ((v) ? 0UL : 0x55555555UL); }

#if 0
static __bit ioreq_pin(void) { return R(Z80_IOREQ); }
static __bit memrq_pin(void) { return R(Z80_MEMRQ); }
static __bit rd_pin(void) { return R(Z80_RD); }
static __bit wr_pin(void) { return R(Z80_WR); }
#endif
static void set_ioreq_pin(uint8_t v) { set_pin(IORQ, v); }
static void set_memrq_pin(uint8_t v) { set_pin(MREQ, v); }
static void set_rd_pin(uint8_t v) { set_pin(RD, v); }
static void set_wr_pin(uint8_t v) { set_pin(WR, v); }
static void set_busrq_pin(uint8_t v) { set_pin(BUSRQ, v); }
static void set_reset_pin(uint8_t v) { set_pin(RESET, v); }
static void set_wait_pin(uint8_t v) { set_pin(WAIT, v); }
static void set_wait_pin_dir(uint8_t v) { set_pin_dir(WAIT, v); }

static void bus_master(int enable)
{
    if (enable) {
        // Set address bus as output
        set_addr_dir(0);            // output

        // Set /IOREQ, /MEMRQ, /RD and /WR as output
        set_ioreq_pin(1);
        set_memrq_pin(1);
        set_rd_pin(1);
        set_wr_pin(1);
        set_wait_pin(1);
        set_pin_dir(IORQ, 0);       // output
        set_pin_dir(MREQ, 0);       // output
        set_pin_dir(RD, 0);         // output
        set_pin_dir(WR, 0);         // output
    } else {
        // Set address and data bus as input
        set_addr_dir(1);            // input
        set_data_dir(1);            // input

        // Set /IOREQ, /MEMRQ, /RD and /WR as input
        set_pin_dir(IORQ, 1);       // input
        set_pin_dir(MREQ, 1);       // input
        set_pin_dir(RD, 1);         // input
        set_pin_dir(WR, 1);         // input
    }
}

void run_clock(int n)
{
    for (int i = 0; i < n; i++) {
        set_pin(CLK, !get_pin(CLK));
        HAL_Delay(1);
        set_pin(CLK, !get_pin(CLK));
        HAL_Delay(1);
    }
}

void board_test(void)
{
    uint8_t prog[] = {
        0xf3,              // DI
        0x3e, 0x0d,        // LD A, \r
        0xd3, 0x5a,        // OUT (5Ah), A
        0x3e, 0x0a,        // LD A, \n
        0xd3, 0x5a,        // OUT (5Ah), A
        0x3e, 0x48,        // LD A, 'H'
        0xd3, 0x5a,        // OUT (5Ah), A
        0x3e, 0x45,        // LD A, 'E'
        0xd3, 0x5a,        // OUT (5Ah), A
        0x3e, 0x4c,        // LD A, 'L'
        0xd3, 0x5a,        // OUT (5Ah), A
        0x3e, 0x4c,        // LD A, 'L'
        0xd3, 0x5a,        // OUT (5Ah), A
        0x3e, 0x4f,        // LD A, 'O'
        0xd3, 0x5a,        // OUT (5Ah), A
        0x3e, 0x0d,        // LD A, \r
        0xd3, 0x5a,        // OUT (5Ah), A
        0x3e, 0x0a,        // LD A, \n
        0xd3, 0x5a,        // OUT (5Ah), A

        0x76,              // HALT
        0x00,              // NOP
    };
    uint8_t buf[sizeof(prog)];
    uint16_t addr;
    uint8_t data;


    /*
     * release reset ...
     */
    run_clock(8);
    set_reset_pin(0);  // reset
    run_clock(8);

    set_pin(BANK_SEL0, 0);  // A16
    set_pin(BANK_SEL1, 1);  // for TC551001 CE2 pin
    set_pin(BANK_SEL2, 0);  // A18

    bus_master(1);

    addr = 0;
    set_memrq_pin(0);
    set_data_dir(0);  // output
    set_addr_pins(addr);
    while (addr < sizeof(prog)) {
        set_data_pins(prog[addr]);
        set_wr_pin(0);  // write enable
        HAL_Delay(1);
        set_wr_pin(1);  // clear write enable
        set_addr_pins(++addr);
        HAL_Delay(1);
    }

    addr = 0;
    set_data_dir(1);  // input
    set_addr_pins(addr);
    while (addr < sizeof(buf)) {
        set_rd_pin(0);  // read enable
        buf[addr] = data_pins();
        HAL_Delay(1);
        set_rd_pin(1);  // clear write enable
        set_addr_pins(++addr);
        HAL_Delay(1);
    }
    set_memrq_pin(1);

    for (addr = 0; addr < sizeof(buf); addr++) {
        if ((addr % 16) == 0)
            printf("read SRAM:");
        printf(" %02X", buf[addr]);
        if ((addr % 16) == 15)
            printf("\r\n");
    }
    if ((addr % 16) != 0)
        printf("\r\n");

    set_busrq_pin(1);
    set_wait_pin_dir(1);  // release wait pin
    bus_master(0);

    /*
     * release reset ...
     */
    run_clock(8);
    set_reset_pin(1);  // release reset

    /*
     * run Z80
     */
    while (1) {
        int i;
        for (i = 0; i < 256; i++) {
            run_clock(1);
            if (get_pin(WAIT) == 0)
                break;
        }
        if (256 <= i) {
            break;
        }

        /*
         * handle I/O request
         */
        if (get_pin(IORQ) == 0) {
            data = data_pins();
            addr = addr_pins();
            if (get_pin(WR) == 0) {
#if 0
                printf("%s: write %02X, %02X %c\r\n", __func__, addr & 0xff, data,
                       (0x30 <= data && data <= 0x7f) ? data : ' ');
#else
                printf("%c", data);
#endif
            } else {
                printf("%s:  read %02X\r\n", __func__, addr & 0xff);
            }
        } else {
            printf("%s: IOREQ is not active\r\n", __func__);
        }

        set_busrq_pin(0);
        set_wait_pin(1);
        set_wait_pin_dir(0);
        run_clock(8);
        set_wait_pin_dir(1);
        run_clock(8);
        set_busrq_pin(1);
        while (get_pin(IORQ) == 0) {
            run_clock(1);
        }
    }

    /*
     * stop Z80
     */
    set_reset_pin(0);  // reset
    run_clock(8);
    bus_master(1);

    printf("%s: halt.\r\n", __func__);
    while (1)
        HAL_Delay(10000);
}
