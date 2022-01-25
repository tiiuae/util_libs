/*
 * Copyright 2021, Technology Innovation Institute
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdint.h>
#include <inttypes.h>
#include <platsupport/plat/aux.h>
#include "../../services.h"

#define AUX_IRQ_MINIUART_IRQ  BIT(0) /* Mini-UART IRQ pending status bit. */
#define AUX_IRQ_SPI1_IRQ      BIT(1) /* AUX SPI1 IRQ pending status bit. */
#define AUX_IRQ_SPI2_IRQ      BIT(2) /* AUX SPI2 IRQ pending status bit. */

#define AUX_ENA_MINIUART_ENA  BIT(0) /* Mini-UART enable/disable bit. */
#define AUX_ENA_SPI1_ENA      BIT(1) /* AUX SPI1 enable/disable bit. */
#define AUX_ENA_SPI2_ENA      BIT(2) /* AUX SPI2 enable/disable bit. */

/* AUX registers definition. */
typedef volatile struct {
    uint32_t    aux_irq;  // 0x00: AUX IRQ pending status
    uint32_t    aux_ena;  // 0x04: AUX device enable/disable
} bcm2711_aux_regs_t;


static struct bcm2711_aux {
    bcm2711_aux_regs_t *regs;
} aux_ctx;

static inline bcm2711_aux_regs_t* aux_get_regs(aux_sys_t* aux_sys){
    return (((struct bcm2711_aux *)aux_sys->priv)->regs);
};


static int bcm2711_aux_get_irq_stat(aux_sys_t *aux_sys, aux_dev_id_t id)
{
    assert(aux_sys);
    assert(aux_sys->priv);

    int ret = 0;
    bcm2711_aux_regs_t* regs = aux_get_regs(aux_sys);

    switch (id)
    {
        case BCM2711_AUX_UART:
            ret = (int) (regs->aux_irq & AUX_IRQ_MINIUART_IRQ);
            break;
        case BCM2711_AUX_SPI1:
            ret = (int) (regs->aux_irq & AUX_IRQ_SPI1_IRQ);
            break;
        case BCM2711_AUX_SPI2:
            ret = (int) (regs->aux_irq & AUX_IRQ_SPI2_IRQ);
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
    bcm2711_aux_regs_t* regs = aux_get_regs(aux_sys);

    switch (id)
    {
        case BCM2711_AUX_UART:
            regs->aux_ena |= AUX_ENA_MINIUART_ENA;
            break;
        case BCM2711_AUX_SPI1:
            regs->aux_ena |= AUX_ENA_SPI1_ENA;
            break;
        case BCM2711_AUX_SPI2:
            regs->aux_ena |= AUX_ENA_SPI2_ENA;
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
    bcm2711_aux_regs_t* regs = aux_get_regs(aux_sys);

    switch (id)
    {
        case BCM2711_AUX_UART:
            regs->aux_ena &= ~AUX_ENA_MINIUART_ENA;
            break;
        case BCM2711_AUX_SPI1:
            regs->aux_ena &= ~AUX_ENA_SPI1_ENA;
            break;
        case BCM2711_AUX_SPI2:
            regs->aux_ena &= ~AUX_ENA_SPI2_ENA;
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
        ZF_LOGF("Failed to map BCM2711 AUX registers frame.");
        return -ENOMEM;
    }

    ZF_LOGD("Mapped AUX registers frame: "
            "paddr / vaddr -> 0x% " PRIxPTR " / 0x% " PRIxPTR,
            (uintptr_t) AUX_PADDR, (uintptr_t) (aux_ctx.regs));

    return bcm2711_aux_init_common(aux_sys);
}

int bcm2711_aux_sys_destroy(const ps_io_ops_t *io_ops, aux_sys_t *aux_sys)
{
    if ((NULL == io_ops) 
    || (NULL == aux_sys)) {
        return -EINVAL;
    }

    if (NULL != aux_sys->priv) {
        ZF_LOGD("Unmapping AUX registers frame: vaddr -> 0x%" PRIxPTR, (uintptr_t) (aux_sys->priv));
        ps_io_unmap(&(io_ops->io_mapper), aux_sys->priv, AUX_SIZE);
    }

    return 0;
}
