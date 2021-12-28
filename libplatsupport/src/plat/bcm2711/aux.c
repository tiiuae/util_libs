/*
 * Copyright 2021, Technology Innovation Institute
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <platsupport/plat/aux.h>
#include "../../services.h"

/* AUX IRQ Register, BCM2711 TRM page 13. */
typedef union {
    struct PACKED {
        unsigned MINIUART_IRQ : 1; /* Mini-UART IRQ pending status bit. */
        unsigned SPI1_IRQ     : 1; /* AUX SPI1 IRQ pending status bit. */
        unsigned SPI2_IRQ     : 1; /* AUX SPI2 IRQ pending status bit. */
        unsigned reserved1    : 29;
    };
    uint32_t raw_32;
} aux_irq_reg_t;

/* AUX ENABles Register, BCM2711 TRM page 14. */
typedef union {
    struct PACKED {
        unsigned MINIUART_ENA : 1; /* Mini-UART enable/disable bit. */
        unsigned SPI1_ENA     : 1; /* AUX SPI1 enable/disable bit. */
        unsigned SPI2_ENA     : 1; /* AUX SPI2 enable/disable bit. */
        unsigned reserved1    : 29;
    };
    uint32_t raw_32;
} aux_enab_reg_t;

/* AUX registers definition. */
typedef volatile struct PACKED_ALIGN(4) {
    const aux_irq_reg_t    aux_irq;        // 0x00: AUX IRQ pending status READ-ONLY
    aux_enab_reg_t         aux_ena;        // 0x04: AUX device enable/disable
} bcm2711_aux_regs_t;


static struct bcm2711_aux {
    bcm2711_aux_regs_t *regs;
} aux_ctx;


#define PRIV_TO_CTX(sys)    ((struct bcm2711_aux *)sys->priv)
#define AUX_GET_REGS(sys)   (((struct bcm2711_aux *)sys->priv)->regs)


static int bcm2711_aux_get_irq_stat(aux_sys_t *aux_sys, aux_dev_id_t id)
{
    assert(aux_sys);
    assert(aux_sys->priv);

    int ret = 0;

    switch (id)
    {
        case BCM2711_AUX_UART:
            ret = (int) AUX_GET_REGS(aux_sys)->aux_irq.MINIUART_IRQ;
            break;
        case BCM2711_AUX_SPI1:
            ret = (int) AUX_GET_REGS(aux_sys)->aux_irq.SPI1_IRQ;
            break;
        case BCM2711_AUX_SPI2:
            ret = (int) AUX_GET_REGS(aux_sys)->aux_irq.SPI2_IRQ;
            break;
        default:
            ZF_LOGE("AUX device ID is not in valid range!");
            ret = -EINVAL;
            break;
    }

    return ret;
}

static int bcm2711_aux_enable(aux_sys_t *aux_sys, aux_dev_id_t id)
{
    assert(aux_sys);
    assert(aux_sys->priv);

    int ret = 0;

    switch (id)
    {
        case BCM2711_AUX_UART:
            AUX_GET_REGS(aux_sys)->aux_ena.MINIUART_ENA = 1;
            break;
        case BCM2711_AUX_SPI1:
            AUX_GET_REGS(aux_sys)->aux_ena.SPI1_ENA = 1;
            break;
        case BCM2711_AUX_SPI2:
            AUX_GET_REGS(aux_sys)->aux_ena.SPI2_ENA = 1;
            break;
        default:
            ZF_LOGE("AUX device ID is not in valid range!");
            ret = -EINVAL;
            break;
    }

    return ret;
}

static int bcm2711_aux_disable(aux_sys_t *aux_sys, aux_dev_id_t id)
{
    assert(aux_sys);
    assert(aux_sys->priv);
    
    int ret = 0;

    switch (id)
    {
        case BCM2711_AUX_UART:
            AUX_GET_REGS(aux_sys)->aux_ena.MINIUART_ENA = 0;
            break;
        case BCM2711_AUX_SPI1:
            AUX_GET_REGS(aux_sys)->aux_ena.SPI1_ENA = 0;
            break;
        case BCM2711_AUX_SPI2:
            AUX_GET_REGS(aux_sys)->aux_ena.SPI2_ENA = 0;
            break;
        default:
            ZF_LOGE("AUX device ID is not in valid range!");
            ret = -EINVAL;
            break;
    }

    return ret;
}

int bcm2711_aux_init_common(aux_sys_t *aux_sys)
{
    assert(aux_sys);

    aux_sys->priv = (void*) &aux_ctx;
    aux_sys->get_irq_stat = &bcm2711_aux_get_irq_stat;
    aux_sys->enable = &bcm2711_aux_enable;
    aux_sys->disable = &bcm2711_aux_disable;

    return 0;
}

int bcm2711_aux_sys_init(const ps_io_ops_t *io_ops, aux_sys_t *aux_sys)
{
    if ((NULL == io_ops) 
    || (NULL == aux_sys)) {
        return -EINVAL;
    }

    MAP_IF_NULL(io_ops, AUX, aux_ctx.regs);
    if(NULL == aux_ctx.regs) {
        ZF_LOGF("Failed to map BCM2711 AUX register frame.");
        return -ENOMEM;
    }

    ZF_LOGD("Mapped AUX registers frame: paddr / vaddr -> 0x%lx / 0x%lx", (uintptr_t) AUX_PADDR, (uintptr_t) (aux_ctx.regs));

    return bcm2711_aux_init_common(aux_sys);
}

int bcm2711_aux_sys_destroy(const ps_io_ops_t *io_ops, aux_sys_t *aux_sys)
{
    if ((NULL == io_ops) 
    || (NULL == aux_sys)) {
        return -EINVAL;
    }

    if (NULL != aux_sys->priv) {
        ZF_LOGD("Unmapping AUX registers frame: vaddr -> 0x%lx", (uintptr_t) (aux_sys->priv));
        ps_io_unmap(&(io_ops->io_mapper), aux_sys->priv, AUX_SIZE);
    }

    return 0;
}
