/*
 * Copyright 2021, Technology Innovation Institute
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <utils/util.h>
#include <platsupport/io.h>

#define BUS_ADDR_OFFSET     0x7E000000
#define PADDR_OFFSET        0xFE000000

/*
 * The AUX block contains 3 auxiliary
 * peripherals on the bcm2711 (section 2.1.)
 *      find in seL4/tools/dts/rpi4.dts under
 *          /soc/aux@7e215000
 *          ->  /soc/serial@7e215040
 *          ->  /soc/spi@7e215080
 *          ->  /soc/spi@7e2150c0
 */
#define AUX_BUSADDR      0x7E215000

#define AUX_PADDR        (AUX_BUSADDR - BUS_ADDR_OFFSET + PADDR_OFFSET)
#define AUX_SIZE         0x1000

/*
 * BCM2711 TRM
 * All AUX device interrupts are on the same interrupt #
 * section 6.2.4. VideoCore interrupts:             29
 * section 6.3.   GIC-400 - VC peripheral IRQs:     96
 * => mini UART IRQ: 96 + 29 = 125
 */
#define AUX_IRQ          (125)

typedef enum aux_dev_id {
    BCM2711_AUX_UART,
    BCM2711_AUX_SPI1,
    BCM2711_AUX_SPI2,

    NUM_AUX_DEV
} aux_dev_id_t;

typedef struct aux_sys aux_sys_t;

struct aux_sys {
    /** Get AUX device IRQ pending status.
     * @param[in]   aux_sys    Initialized AUX driver instance.
     * @param[in]   id         ID of the AUX device.
     * @return 0/1 on success. Non-zero on error.
     */
    int (*get_irq_stat)(aux_sys_t *aux_sys, aux_dev_id_t id);

    /**
     * Enable AUX device.
     * @param[in]   aux_sys    Initialized AUX driver instance.
     * @param[in]   id         ID of the AUX device.
     * @return 0 on success. Non-zero on error.
     */
    int (*enable)(aux_sys_t *aux_sys, aux_dev_id_t id);

    /**
     * Disable AUX device. Disabling an AUX device also
     * disables access to the respective devices' registers.
     * @param[in]   aux_sys    Initialized AUX driver instance.
     * @param[in]   id         ID of the AUX device.
     * @return 0 on success. Non-zero on error.
     */
    int (*disable)(aux_sys_t *aux_sys, aux_dev_id_t id);

/// platform specific private data
    void *priv;
};

/**
 * Initialise the BCM2711 AUX system handler instance.
 * @param[in]  io_ops    IO operations for device initialisation.
 * @param[out] aux_sys   A handle to a AUX subsystem to populate.
 * @return               0 on success. Non-zero on error.
 */
int bcm2711_aux_sys_init(const ps_io_ops_t *io_ops, aux_sys_t *aux_sys);

/**
 * De-initialise the BCM2711 AUX system handler instance.
 * @param[in]  io_ops    IO operations for device initialisation.
 * @param[out] aux_sys   A handle to a AUX subsystem to populate.
 * @return               0 on success. Non-zero on error.
 */
int bcm2711_aux_sys_destroy(const ps_io_ops_t *io_ops, aux_sys_t *aux_sys);
