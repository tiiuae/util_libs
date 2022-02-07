/*
 * Copyright (C) 2021, Hensoldt Cyber GmbH
 * Copyright 2022, Technology Innovation Institute
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <platsupport/serial.h>
#include <platsupport/plat/aux.h>
#include <platsupport/plat/serial.h>
#include "serial.h"
#include "mini_serial.h"

/*
 * The Raspberry Pi's mini-UART/AUX UART has some shenanigans
 * going on as the TRM is not correct on many parts. The UART
 * device works essentially the same as on BCM2837/BCM2835, so
 * the same erratas apply here.
 *
 * See following resources for details:
 *     BCM2711 TRM, Section 2.2. Mini UART.
 *     https://elinux.org/BCM2835_datasheet_errata
 *     https://elinux.org/RPi_Hardware
 */

/* BCM2711 TRM page 17. Interrupt ID definitions. */
enum mu_iir_source {
    MU_NO_INTERRUPTS = 0, /* No interrupts pending. */
    MU_TXE_INTERRUPT = 1, /* TX FIFO empty causing interrupt. */
    MU_RXD_INTERRUPT = 2  /* RX FIFO data received causing interrupt. */
};

/* BCM2711 TRM pages 20-21. Auto-flow-control RTS threshold definitions. */
enum mu_cntl_flow_level {
    FIFO_3B_LEFT = 0,  /* RTS de-asserted when RX FIFO has 3 bytes left. */
    FIFO_2B_LEFT = 1,  /* RTS de-asserted when RX FIFO has 2 bytes left. */
    FIFO_1B_LEFT = 2,  /* RTS de-asserted when RX FIFO has 1 bytes left. */
    FIFO_4B_LEFT = 3   /* RTS de-asserted when RX FIFO has 4 bytes left. */
};

#define MU_FIFO_SIZE          (8) /* 8-byte FIFOs for RX & TX. */

#define MU_IO_DATA_MASK       MASK(8) /* IO register data bits mask. (Bits 7:0). */

/* TRM has errors regarding IER register, see errata comment above */
#define MU_IER_RXD_IRQ        BIT(0) /* Interrupt if RX FIFO holds at least 1 byte. */
#define MU_IER_TXE_IRQ        BIT(1) /* Interrupt if TX FIFO is empty. */
#define MU_IER_LSI_IRQ        BIT(2) /* RX Line Status Interrupt on overrun/parity/framing errors etc. */
#define MU_IER_MSI_IRQ        BIT(3) /* Modem Status Interrupt on changing states to either: To Send(CTS), Data Set Ready(DSR). */

#define MU_IIR_PENDING        BIT(0) /* This bit is clear if there is an interrupt pending. */
#define MU_IIR_SOURCE_MASK    (MASK(3) & ~BIT(0)) /* Interrupt source bits mask. (Bits 2:1). */
#define MU_IIR_SOURCE_SHIFT   (1)
#define MU_IIR_RXF_CLEAR      BIT(1) /* RX FIFO clear. */
#define MU_IIR_TXF_CLEAR      BIT(2) /* TX FIFO clear. */

#define MU_LCR_DATASIZE       BIT(0) /* 0 -> 7-bit mode, 1 -> 8-bit mode. */
#define MU_LCR_BREAK          BIT(6) /* This bit is set if TX line is pulled low for at least 12 bit times. (Break condition).*/
#define MU_LCR_DLAB           BIT(7) /* DLAB access bit. */

#define MU_MCR_RTS            BIT(1) /* 0 -> RTS is high, 1 -> RTS is low. */

#define MU_LSR_RXF_DATAREADY  BIT(0) /* This bit is set when RX FIFO holds at least one byte. */
#define MU_LSR_RX_OVERRUN     BIT(1) /* This bit is set when RX FIFO was overrun. */
#define MU_LSR_TXF_HASSPACE   BIT(5) /* This bit is set if the TX FIFO can accept at least one byte.*/
#define MU_LSR_TXIDLE         BIT(6) /* This bit is set if the TX FIFO is empty and the TX is idle. (Finished shifting out the last bit). */

#define MU_MSR_CTS            BIT(5) /* 0 -> CTS is high, 1 -> CTS is low. */

#define MU_CNTL_RXE           BIT(0) /* Receiver (RX) enable. */
#define MU_CNTL_TXE           BIT(1) /* Transmitter (TX) enable. */
#define MU_CNTL_RTSEN         BIT(2) /* 1 -> RTS line will de-assert if RX FIFO reaches auto-flow level, 0 -> RTS is controlled by the AUX_MU_MCR_REG register bit 1. */
#define MU_CNTL_CTSEN         BIT(3) /* 1 -> TX will stop when CTS line is de-asserted, 0 -> CTS line is ignored. */
#define MU_CNTL_RTS_LVL_MASK  (MASK(5) & ~MASK(3)) /* RTS auto-flow-control bits, specifies the # of bytes left in RX FIFO until RTS is de-asserted. */
#define MU_CNTL_RTS_LVL_SHIFT (3)
#define MU_CNTL_RTS_INV       BIT(6) /* 1 -> RTS auto flow assert level is low, 0 -> RTS auto flow assert level is high. */
#define MU_CNTL_CTS_INV       BIT(7) /* 1 -> CTS auto flow assert level is low, 0 -> CTS auto flow assert level is high. */

#define MU_STAT_RXF_DATAREADY BIT(0) /* This bit is set when RX FIFO holds at least one byte. */
#define MU_STAT_TXF_HASSPACE  BIT(1) /* This bit is set if the TX FIFO can accept at least one byte.*/
#define MU_STAT_RXIDLE        BIT(2) /* This bit is set if the receiver is at idle. */
#define MU_STAT_TXIDLE        BIT(3) /* This bit is set if the transmitter is at idle. */
#define MU_STAT_RX_OVERRUN    BIT(4) /* This bit is set when RX FIFO was overrun. */
#define MU_STAT_TXF_FULL      BIT(5) /* Inverse of bit 1. This bit is set if TX FIFO is completely full. */
#define MU_STAT_RTS           BIT(6) /* RTS line status. */
#define MU_STAT_CTS           BIT(7) /* CTS line status. */
#define MU_STAT_TXF_EMPTY     BIT(8) /* This bit is set if TX FIFO is completely empty (can accept 8 bytes). */
#define MU_STAT_TXDONE        BIT(9) /* This bit is set if TX FIFO is completely empty and the transmitter is at idle. (AND of bits 3 & 8). */
#define MU_STAT_RXF_LVL_MASK  ((MASK(20) & ~MASK(16))) /* # of bytes in RX FIFO. (0..8). */
#define MU_STAT_RXF_LVL_SHIFT (16)
#define MU_STAT_TXF_LVL_MASK  ((MASK(28) & ~MASK(24))) /* # of bytes in TX FIFO. (0..8). */
#define MU_STAT_TXF_LVL_SHIFT (24)

#define MU_BAUD_BAUD_MASK     MASK(16) /* 16-bits baudrate value. */

/* Mini-UART registers definition. */
typedef volatile struct mini_uart_regs_s {
    uint32_t mu_io;         // 0x40: mini UART I/O Data
    uint32_t mu_ier;        // 0x44: mini UART interrupt enable
    uint32_t mu_iir;        // 0x48: mini UART interrupt identify
    uint32_t mu_lcr;        // 0x4c: mini UART line control
    uint32_t mu_mcr;        // 0x50: mini UART modem control
    uint32_t mu_lsr;        // 0x54: mini UART line status
    uint32_t mu_msr;        // 0x58: mini UART modem status
    uint32_t mu_scratch;    // 0x5c: mini UART scratch
    uint32_t mu_cntl;       // 0x60: mini UART extra control
    uint32_t mu_stat;       // 0x64: mini UART extra status
    uint32_t mu_baud;       // 0x68: mini UART baudrate
}
mini_uart_regs_t;

static aux_sys_t aux;

static inline mini_uart_regs_t *mini_uart_get_priv(ps_chardevice_t *dev)
{
    return ((mini_uart_regs_t *)dev->vaddr);
}

static inline int mini_uart_flush_fifos(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);

    regs->mu_iir |= (MU_IIR_RXF_CLEAR | MU_IIR_TXF_CLEAR);

    return 0;
}

static inline int mini_uart_enable(ps_chardevice_t *dev, aux_sys_t *aux)
{
    if (dev == NULL ||
        aux == NULL) {
        return -EINVAL;
    }

    int ret = 0;

    ret = aux->enable(aux, BCM2711_AUX_UART);

    if (ret != 0) {
        ZF_LOGE("Failed to enable mini-UART in AUX subsystem! %i", ret);
        return ret;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    regs->mu_cntl |= (MU_CNTL_RXE | MU_CNTL_TXE);

    return 0;
}

static inline int mini_uart_disable(ps_chardevice_t *dev, aux_sys_t *aux, bool disable_aux)
{
    if (dev == NULL ||
        aux == NULL) {
        return -EINVAL;
    }

    int ret = 0;

    if (disable_aux) {

        ret = aux->disable(aux, BCM2711_AUX_UART);

        if (ret != 0) {
            ZF_LOGE("Failed to disable mini-UART in AUX subsystem! %i", ret);
            return ret;
        }
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    regs->mu_cntl &= ~(MU_CNTL_RXE | MU_CNTL_TXE);

    return 0;
}

static inline int mini_uart_enable_rx_irq(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    regs->mu_ier |= (MU_IER_RXD_IRQ | MU_IER_LSI_IRQ);

    return 0;
}

static inline int mini_uart_disable_rx_irq(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    regs->mu_ier &= ~(MU_IER_RXD_IRQ | MU_IER_LSI_IRQ);

    return 0;
}

static inline int mini_uart_enable_tx_irq(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    regs->mu_ier |= (MU_IER_TXE_IRQ | MU_IER_LSI_IRQ);

    return 0;
}

static inline int mini_uart_disable_tx_irq(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    regs->mu_ier &= ~(MU_IER_TXE_IRQ | MU_IER_LSI_IRQ);

    return 0;
}

static inline int mini_uart_disable_interrupts(ps_chardevice_t *dev)
{
    return (mini_uart_disable_rx_irq(dev) | mini_uart_disable_tx_irq(dev));
}

static inline int mini_uart_get_irq_source(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    return (int)((regs->mu_iir & MU_IIR_SOURCE_MASK) >> MU_IIR_SOURCE_SHIFT);
}

static inline int mini_uart_rxfifo_level(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    return (int)((regs->mu_stat & MU_STAT_RXF_LVL_MASK) >> MU_STAT_RXF_LVL_SHIFT);
}

static inline int mini_uart_txfifo_level(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);
    return (int)((regs->mu_stat & MU_STAT_TXF_LVL_MASK) >> MU_STAT_TXF_LVL_SHIFT);
}


static int mini_uart_putchar_blocking(ps_chardevice_t *d, int c)
{
    if (d == NULL) {
        return -EINVAL;
    }

    const char ch = (const char)c;
    mini_uart_regs_t *regs = mini_uart_get_priv(d);

    int slots_required = 1;
    bool print_cr = false;
    bool complete = false;

    if ((d->flags & SERIAL_AUTO_CR) && (ch == '\n')) {
        slots_required = 2;
        print_cr = true;
    }

    do {
        int slots_avail = (MU_FIFO_SIZE - mini_uart_txfifo_level(d));

        if (slots_avail >= slots_required) {

            regs->mu_io = ch;
            if (print_cr) {
                regs->mu_io = '\r';
            }

            complete = true;
        }
    } while (!complete);

    return 0;
}

static int mini_uart_putchar_nonblocking(ps_chardevice_t *d, int c)
{
    if (d == NULL) {
        return -EINVAL;
    }

    const char ch = (const char)c;
    mini_uart_regs_t *regs = mini_uart_get_priv(d);

    int slots_required = 1;
    bool print_cr = false;
    bool complete = false;

    if ((d->flags & SERIAL_AUTO_CR) && (ch == '\n')) {
        slots_required = 2;
        print_cr = true;
    }

    int slots_avail = (MU_FIFO_SIZE - mini_uart_txfifo_level(d));

    if (slots_avail < slots_required) {
        return EOF;
    } else {
        regs->mu_io = ch;
        if (print_cr) {
            regs->mu_io = '\r';
        }
    }

    return 0;
}

static int mini_uart_getchar_blocking(ps_chardevice_t *d)
{
    if (d == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(d);

    while (!(regs->mu_stat & MU_STAT_RXF_DATAREADY)) {
        return (int)(regs->mu_io & MU_IO_DATA_MASK);
    }
}

static int mini_uart_getchar_nonblocking(ps_chardevice_t *d)
{
    if (d == NULL) {
        return -EINVAL;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(d);

    if (regs->mu_stat & MU_STAT_RXF_DATAREADY) {
        return (int)(regs->mu_io & MU_IO_DATA_MASK);
    }

    return EOF;
}

static void mini_uart_handle_irq(ps_chardevice_t *dev)
{
    if (dev == NULL) {
        return;
    }

    /*
     * TODO: how to handle TX interrupts?
     */

    const int source = mini_uart_get_irq_source(dev);
    ZF_LOGD("mini-UART interrupt source: %i", source);

    if (MU_RXD_INTERRUPT == source) {
        mini_uart_enable_rx_irq(dev);
    } else if (MU_TXE_INTERRUPT == source) {
        ZF_LOGD("Unhandled TXE interrupt");
    }
}

int mini_uart_init(const struct dev_defn *defn,
                   const ps_io_ops_t *ops,
                   ps_chardevice_t *dev)
{
    /* Attempt to map the virtual address, assure this works */
    void *vaddr = chardev_map(defn, ops);
    memset(dev, 0, sizeof(*dev));
    if (vaddr == NULL) {
        return -ENOMEM;
    }

    // When mapping the virtual address space, the base addresses need to be
    // 4k byte aligned. Since the real base addresses of the UART peripherals
    // are not 4k byte aligned, we have to add the required offset.
    uint32_t addr_offset = 0;
    switch (defn->id) {
    case 1:
        addr_offset = UART1_OFFSET;
        break;
    default:
        ZF_LOGE("Mini UART with ID %d does not exist!", defn->id);
        return -EINVAL;
        break;
    }

    /* Set up all the  device properties. */
    dev->id         = defn->id;
    dev->vaddr      = (void *)vaddr + addr_offset; // use real base address
    dev->read       = &uart_read;
    dev->write      = &uart_write;
    dev->handle_irq = &mini_uart_handle_irq;
    dev->irqs       = defn->irqs;
    dev->ioops      = *ops;
    dev->flags      = SERIAL_AUTO_CR;

    int ret = 0;

    /* Initialize AUX subsystem handle */
    ret = bcm2711_aux_sys_init(ops, &aux);
    if (ret != 0) {
        ZF_LOGE("Failed to initialize AUX subsystem! %i", ret);
        return ret;
    }

    mini_uart_regs_t *regs = mini_uart_get_priv(dev);

    /* Disable RX/TX and interrupts */
    ret = mini_uart_disable(dev, &aux, false);
    if (ret != 0) {
        ZF_LOGE("Failed to disable mini-UART! %i", ret);
        return ret;
    }

    /* Set databits to 8 and ensure DLAB == 0 */
    regs->mu_lcr |= MU_LCR_DATASIZE;
    regs->mu_lcr &= ~MU_LCR_DLAB;

    /* Flush FIFO buffers and enable mini-UART */
    ret = mini_uart_flush_fifos(dev);
    if (ret != 0) {
        ZF_LOGE("Failed to flush mini-UART FIFOs! %i", ret);
        return ret;
    }

    ret = mini_uart_enable(dev, &aux);
    if (ret != 0) {
        ZF_LOGE("Failed to enable mini-UART! %i", ret);
        return ret;
    }

    return 0;
}

int mini_uart_getchar(ps_chardevice_t *d)
{
    if (d == NULL) {
        return -EINVAL;
    }

#if 0
    if (d->flags & SERIAL_RX_NONBLOCKING) {
        return mini_uart_getchar_nonblocking(d);
    } else {
        return mini_uart_getchar_blocking(d);
    }
#else
    return mini_uart_getchar_nonblocking(d);
#endif
}

int mini_uart_putchar(ps_chardevice_t *d, int c)
{
    if (d == NULL) {
        return -EINVAL;
    }

#if 0
    if (d->flags & SERIAL_TX_NONBLOCKING) {
        return mini_uart_putchar_nonblocking(d, c);
    } else {
        return mini_uart_putchar_blocking(d, c);
    }
#else
    return mini_uart_putchar_blocking(d, c);
#endif
}
