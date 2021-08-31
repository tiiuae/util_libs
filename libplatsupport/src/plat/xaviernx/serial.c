/*
 * Copyright 2021, Technology Innovation Institute
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <string.h>
#include <stdlib.h>
#include <platsupport/serial.h>
#include "../../chardev.h"

#if 0
#define RHR         0x00
#define THR         0x00
#define IER         0x04
#define LSR         0x14
#define RHR_MASK    MASK(8)
#define IER_RHRIT   BIT(0)
#define LSR_TXFIFOE BIT(5)
#define LSR_RXFIFOE BIT(0)

#define REG_PTR(base, off)     ((volatile uint32_t *)((base) + (off)))
#endif

int uart_getchar(ps_chardevice_t *d)
{
#if 0
    int ch = EOF;

    if (*REG_PTR(d->vaddr, LSR) & LSR_RXFIFOE) {
        ch = *REG_PTR(d->vaddr, RHR) & RHR_MASK;
    }
    return ch;
#endif
    return EOF;
}

int uart_putchar(ps_chardevice_t* d, int c)
{
	volatile uint32_t *mb = ((uint32_t *) d->vaddr);

        while (*mb & 0x80000000);

        *mb = ((uint32_t) c) |
                (0x81 << 24);

        while (*mb & 0x80000000);
}

static void
uart_handle_irq(ps_chardevice_t* d UNUSED)
{
    /* nothing to do */
}

int uart_init(const struct dev_defn* defn,
              const ps_io_ops_t* ops,
              ps_chardevice_t* dev)
{
    memset(dev, 0, sizeof(*dev));
    void* vaddr = chardev_map(defn, ops);
    if (vaddr == NULL) {
        return -1;
    }

    /* Set up all the  device properties. */
    dev->id         = defn->id;
    dev->vaddr      = (void*)vaddr;
    dev->read       = &uart_read;
    dev->write      = &uart_write;
    dev->handle_irq = &uart_handle_irq;
    dev->irqs       = defn->irqs;
    dev->ioops      = *ops;
    dev->flags      = SERIAL_AUTO_CR;

    return 0;
}
