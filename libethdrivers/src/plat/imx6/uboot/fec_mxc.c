/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2020, HENSOLDT Cyber GmbH
 */

/*
 * (C) Copyright 2009 Ilya Yanok, Emcraft Systems Ltd <yanok@emcraft.com>
 * (C) Copyright 2008,2009 Eric Jarrige <eric.jarrige@armadeus.org>
 * (C) Copyright 2008 Armadeus Systems nc
 * (C) Copyright 2007 Pengutronix, Sascha Hauer <s.hauer@pengutronix.de>
 * (C) Copyright 2007 Pengutronix, Juergen Beisert <j.beisert@pengutronix.de>
 * (C) Copyright 2018, NXP
 * (C) Copyright 2020, HENSOLDT Cyber GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "common.h"
#include "miiphy.h"
#include "fec_mxc.h"

#include "imx-regs.h"
#include "../io.h"

#include "micrel.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "../enet.h"
#include "../ocotp_ctrl.h"

#undef DEBUG

/*----------------------------------------------------------------------------*/
static int
fec_phy_read(
    struct mii_dev *bus,
    int phyAddr,
    UNUSED int dev_addr,
    int regAddr)
{
    struct enet *enet = (struct enet *)bus->priv;
    assert(enet);
    return enet_mdio_read(enet, phyAddr, regAddr);
}

/*----------------------------------------------------------------------------*/
static int
fec_phy_write(
    struct mii_dev *bus,
    int phyAddr,
    UNUSED int dev_addr,
    int regAddr,
    uint16_t data)
{
    struct enet *enet = (struct enet *)bus->priv;
    assert(enet);
    return enet_mdio_write(enet, phyAddr, regAddr, data);
}

// /*---------------------------------------------------------------------------
//  * Halt the FEC engine
//  * @param[in] dev Our device to handle
//  */
// void
// fec_halt(
//     struct eth_device *dev)
// {
//     struct fec_priv *fec = (struct fec_priv *)dev->priv;
//     int counter = 0xffff;
//     /* issue graceful stop command to the FEC transmitter if necessary */
//     writel(FEC_TCNTRL_GTS | readl(&fec->eth->x_cntrl), &fec->eth->x_cntrl);
//     /* wait for graceful stop to register */
//     while ((counter--) && (!(readl(&fec->eth->ievent) & FEC_IEVENT_GRA))) {
//         udelay(1);
//     }
//     writel(readl(&fec->eth->ecntrl) & ~FEC_ECNTRL_ETHER_EN, &fec->eth->ecntrl);
//     fec->rbd_index = 0;
//     fec->tbd_index = 0;
// }

/*----------------------------------------------------------------------------*/
struct phy_device *
fec_init(
    unsigned int phy_mask,
    struct enet *enet)
{
    int ret;

    /* Allocate the mdio bus */
    struct mii_dev *bus = mdio_alloc();
    if (!bus) {
        LOG_ERROR("Could not allocate MDIO");
        return NULL;
    }
    strncpy(bus->name, "MDIO", sizeof(bus->name));
    bus->read = fec_phy_read;
    bus->write = fec_phy_write;
    bus->priv = enet;
    ret = mdio_register(bus);
    if (ret) {
        LOG_ERROR("Could not register MDIO, code %d", ret);
        free(bus);
        return NULL;
    }

    /* ***** Configure phy *****/
    struct eth_device edev = { .name = "DUMMY-EDEV" }; // just a dummy
    struct phy_device *phydev= phy_connect_by_mask(
                                bus,
                                phy_mask,
                                &edev,
                                PHY_INTERFACE_MODE_RGMII);
    if (!phydev) {
        LOG_ERROR("Could not connect to PHY");
        return NULL;
    }

#if defined(CONFIG_PLAT_IMX8MQ_EVK)

    /* enable rgmii rxc skew and phy mode select to RGMII copper */
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
    phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

    if (phydev->drv->config) {
        phydev->drv->config(phydev);
    }

    if (phydev->drv->startup) {
        phydev->drv->startup(phydev);
    }

#elif defined(CONFIG_PLAT_IMX6)

    /* min rx data delay */
    ksz9021_phy_extended_write(phydev, MII_KSZ9021_EXT_RGMII_RX_DATA_SKEW, 0x0);
    /* min tx data delay */
    ksz9021_phy_extended_write(phydev, MII_KSZ9021_EXT_RGMII_TX_DATA_SKEW, 0x0);
    /* max rx/tx clock delay, min rx/tx control */
    ksz9021_phy_extended_write(phydev, MII_KSZ9021_EXT_RGMII_CLOCK_SKEW, 0xf0f0);
    ksz9021_config(phydev);

    /* Start up the PHY */
    ret = ksz9021_startup(phydev);
    if (ret) {
        LOG_ERROR("Could not initialize PHY '%s', code %d", phydev->dev->name, ret);
        return NULL;
    }

#else
#error "unsupported platform"
#endif

    return phydev;
}
