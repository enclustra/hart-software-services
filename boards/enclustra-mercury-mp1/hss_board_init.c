/*******************************************************************************
 * Copyright 2017-2022 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HSS Embedded Software
 *
 */

/**
 * \file HSS Board Initalization
 * \brief Board Initialization
 */

#include "config.h"
#include "hss_types.h"
#include <assert.h>
#include <string.h>

#include "hss_debug.h"
#include "mss_ethernet_registers.h"
#include "mss_ethernet_mac_regs.h"
#include "mss_gpio.h"
#include "hss_init.h"
#include "hss_state_machine.h"
#include "ssmb_ipi.h"
#include "hss_registry.h"
#include "ddr_service.h"
#include "ddr/hw_ddrc.h"

/******************************************************************************************************/
/*!
 * \brief Board Init Function Registration Table
 *
 * The following structure is used to connect in new board init functions.
 */

#include "hss_init.h"
#include "hss_boot_pmp.h"
#include "hss_sys_setup.h"
#include "hss_board_init.h"

const struct InitFunction /*@null@*/ boardInitFunctions[] = {
    // Name                     FunctionPointer         Halt   Restart
    { "HSS_ZeroTIMs",           HSS_ZeroTIMs,           false, false },
    { "HSS_Setup_PLIC",         HSS_Setup_PLIC,         false, false },
    { "HSS_Setup_BusErrorUnit", HSS_Setup_BusErrorUnit, false, false },
    { "HSS_Setup_MPU",          HSS_Setup_MPU,          false, false },
    { "HSS_DDRInit",            HSS_DDRInit,            false, false },
#ifdef CONFIG_USE_PCIE
    { "HSS_PCIeInit",           HSS_PCIeInit,           false, false },
#endif
    { "HSS_USBInit",            HSS_USBInit,            false, false },
};

/******************************************************************************************************/

/**
 * \brief Board Initialization Function
 *
 * All other initialization routines to be chained off this...
 */

/****************************************************************************/

#define MSS_MAC1_BASE (0x20112000U)

void ENC_init_mdio(MAC_TypeDef *mac_base);
void ENC_wait_for_mdio_idle(MAC_TypeDef *mac_base);
void ENC_write_phy_reg(MAC_TypeDef *mac_base, uint8_t phyaddr, uint8_t regaddr, uint16_t regval);
uint16_t ENC_read_phy_reg(MAC_TypeDef *mac_base, uint8_t phyaddr, uint8_t regaddr);
void ENC_InitializeMemory(uint64_t *addr, uint32_t size);

void ENC_init_mdio(MAC_TypeDef *mac_base)
{
    mac_base->NETWORK_CONTROL = GEM_MAN_PORT_EN | GEM_CLEAR_ALL_STATS_REGS;
}

void ENC_wait_for_mdio_idle(MAC_TypeDef *mac_base)
{
    do
    {
        volatile int32_t ix;
        ix++;
    } while(0U == (mac_base->NETWORK_STATUS & GEM_MAN_DONE));
}

void ENC_write_phy_reg(MAC_TypeDef *mac_base, uint8_t phyaddr, uint8_t regaddr, uint16_t regval)
{
    ENC_wait_for_mdio_idle(mac_base);

    volatile uint32_t phy_op;
    phy_op = GEM_WRITE1 | (GEM_PHY_OP_CL22_WRITE << GEM_OPERATION_SHIFT) | (((uint32_t)(2UL)) << GEM_WRITE10_SHIFT) | (uint32_t)regval;
    phy_op |= ((uint32_t)phyaddr << GEM_PHY_ADDRESS_SHIFT) & GEM_PHY_ADDRESS;
    phy_op |= ((uint32_t)regaddr << GEM_REGISTER_ADDRESS_SHIFT) & GEM_REGISTER_ADDRESS;

    mac_base->PHY_MANAGEMENT = phy_op;
}

uint16_t ENC_read_phy_reg(MAC_TypeDef *mac_base, uint8_t phyaddr, uint8_t regaddr)
{
    ENC_wait_for_mdio_idle(mac_base);

    volatile uint32_t phy_op;
    phy_op = GEM_WRITE1 | (GEM_PHY_OP_CL22_READ << GEM_OPERATION_SHIFT) | (((uint32_t)(2UL)) << GEM_WRITE10_SHIFT);
    phy_op |= ((uint32_t)phyaddr << GEM_PHY_ADDRESS_SHIFT) & GEM_PHY_ADDRESS;
    phy_op |= ((uint32_t)regaddr << GEM_REGISTER_ADDRESS_SHIFT) & GEM_REGISTER_ADDRESS;

    mac_base->PHY_MANAGEMENT = phy_op;
    ENC_wait_for_mdio_idle(mac_base);

    phy_op = mac_base->PHY_MANAGEMENT;

    return((uint16_t)phy_op);
}

bool HSS_BoardInit(void)
{
    RunInitFunctions(ARRAY_SIZE(boardInitFunctions), boardInitFunctions);

    return true;
}

void ENC_InitializeMemory(uint64_t *addr, uint32_t size)
{
    mHSS_FANCY_PRINTF(LOG_NORMAL, "Initializing memory offset 0x%x%08x size 0x%x\n", (uint64_t)addr >> 32, addr, size);
    memset(addr, 0, size);
}

bool HSS_BoardLateInit(void)
{
    // Make sure peripheral reset is released
    MSS_GPIO_init(GPIO0_LO);
    MSS_GPIO_set_output(GPIO0_LO, MSS_GPIO_12, 1);
    MSS_GPIO_config(GPIO0_LO, MSS_GPIO_12, MSS_GPIO_OUTPUT_MODE);

    // DP83867IS requires 200ms between reset and MDIO access
    HSS_SpinDelay_MilliSecs(250);

    MAC_TypeDef *mac_base = (MAC_TypeDef*)MSS_MAC1_BASE;
    ENC_init_mdio(mac_base);

    // Configure pin INT#/PWDN# of both DP83867IS Ethernet PHY for interrupt functionality
    ENC_write_phy_reg(mac_base, 0, 0x1e, 0x80);
    ENC_write_phy_reg(mac_base, 3, 0x1e, 0x80);

    // Clear power down flag
    ENC_write_phy_reg(mac_base, 0, 0x0, 0x1140);
    ENC_write_phy_reg(mac_base, 3, 0x0, 0x1140);

    // Check if PHY configuration was successful
    uint16_t reg = ENC_read_phy_reg(mac_base, 0x0, 0x0);
    if (reg != 0x1140)
    {
        mHSS_DEBUG_PRINTF(LOG_WARN, "Phy configuration error\n");
    }
    reg = ENC_read_phy_reg(mac_base, 0x3, 0x0);
    if (reg != 0x1140)
    {
        mHSS_DEBUG_PRINTF(LOG_WARN, "Phy configuration error\n");
    }

    // With ECC enabled, the DDR memory needs to be initialized to prevent from
    // bus errors caused by reading uninitialized memory
#if LIBERO_SETTING_CFG_ECC_CORRECTION_EN == 1
    ENC_InitializeMemory((uint64_t *)HSS_DDR_GetStart(), HSS_DDR_GetSize());
    ENC_InitializeMemory((uint64_t *)HSS_DDRHi_GetStart(), HSS_DDRHi_GetSize());
#endif

    return true;
}
