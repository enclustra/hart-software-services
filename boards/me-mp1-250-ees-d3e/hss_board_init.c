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

#include "hss_debug.h"
#include "mss_ethernet_registers.h"
#include "mss_ethernet_mac_regs.h"
#include "mss_gpio.h"
#include "hss_init.h"
#include "hss_state_machine.h"
#include "ssmb_ipi.h"
#include "hss_registry.h"
#include "hal/hal.h"

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
    { "HSS_ZeroDDR",            HSS_ZeroDDR,            false, false },
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
void ENC_write_phy_reg(uint8_t phyaddr, uint8_t regaddr, uint16_t regval);
uint16_t ENC_read_phy_reg(uint8_t phyaddr, uint8_t regaddr);
void ENC_sleep_ms(unsigned int count);
void ENC_wait_for_mdio_idle(void);

#include "mss_sysreg.h"
bool HSS_BoardInit(void)
{
    RunInitFunctions(ARRAY_SIZE(boardInitFunctions), boardInitFunctions);

    return true;
}

// TODO: there must be a better solution
void ENC_sleep_ms(unsigned int count)
{
    const uint64_t ms_count = 16493;
    volatile int64_t sleep = 0;
    volatile int64_t end = count * ms_count;
    do
    {
        sleep++;
    } while(sleep < end);
}

void ENC_wait_for_mdio_idle(void)
{
    MAC_TypeDef *mac_base = (MAC_TypeDef*)MSS_MAC1_BASE;
    do
    {
        volatile int32_t ix;
        ix++;
    } while(0U == (mac_base->NETWORK_STATUS & GEM_MAN_DONE));
}

void ENC_write_phy_reg(uint8_t phyaddr, uint8_t regaddr, uint16_t regval)
{
    MAC_TypeDef *mac_base = (MAC_TypeDef*)MSS_MAC1_BASE;
    psr_t lev = HAL_disable_interrupts();

    mac_base->NETWORK_CONTROL = GEM_MAN_PORT_EN | GEM_CLEAR_ALL_STATS_REGS;

    ENC_wait_for_mdio_idle();

    volatile uint32_t phy_op;
    phy_op = GEM_WRITE1 | (GEM_PHY_OP_CL22_WRITE << GEM_OPERATION_SHIFT) | (((uint32_t)(2UL)) << GEM_WRITE10_SHIFT) | (uint32_t)regval;
    phy_op |= ((uint32_t)phyaddr << GEM_PHY_ADDRESS_SHIFT) & GEM_PHY_ADDRESS;
    phy_op |= ((uint32_t)regaddr << GEM_REGISTER_ADDRESS_SHIFT) & GEM_REGISTER_ADDRESS;

    mac_base->PHY_MANAGEMENT = phy_op;
    HAL_restore_interrupts(lev);
}

uint16_t ENC_read_phy_reg(uint8_t phyaddr, uint8_t regaddr)
{
    MAC_TypeDef *mac_base = (MAC_TypeDef*)MSS_MAC1_BASE;

    psr_t lev = HAL_disable_interrupts();

    mac_base->NETWORK_CONTROL = GEM_MAN_PORT_EN | GEM_CLEAR_ALL_STATS_REGS;
    ENC_wait_for_mdio_idle();

    volatile uint32_t phy_op;
    phy_op = GEM_WRITE1 | (GEM_PHY_OP_CL22_READ << GEM_OPERATION_SHIFT) | (((uint32_t)(2UL)) << GEM_WRITE10_SHIFT);
    phy_op |= ((uint32_t)phyaddr << GEM_PHY_ADDRESS_SHIFT) & GEM_PHY_ADDRESS;
    phy_op |= ((uint32_t)regaddr << GEM_REGISTER_ADDRESS_SHIFT) & GEM_REGISTER_ADDRESS;

    mac_base->PHY_MANAGEMENT = phy_op;
    ENC_wait_for_mdio_idle();

    phy_op = mac_base->PHY_MANAGEMENT;
    HAL_restore_interrupts(lev);

    return((uint16_t)phy_op);
}

bool HSS_BoardLateInit(void)
{
    // Reset peripherals
    MSS_GPIO_init(GPIO0_LO);
    MSS_GPIO_config(GPIO0_LO, MSS_GPIO_12, MSS_GPIO_OUTPUT_MODE);
    MSS_GPIO_set_output(GPIO0_LO, MSS_GPIO_12, 1);

    // Wait until both Ethernet PHYs are ready
    ENC_sleep_ms(10);

    // Configure pin INT#/PWDN# of both DP83867IS Ethernet PHY for interrupt functionality
    ENC_write_phy_reg(0, 0x1e, 0x80);
    ENC_write_phy_reg(3, 0x1e, 0x80);

    // Clear power down flag
    ENC_write_phy_reg(0, 0x0, 0x1140);
    ENC_write_phy_reg(3, 0x0, 0x1140);

    return true;
}
