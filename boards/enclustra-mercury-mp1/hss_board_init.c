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
#define UART_SEL_GPIO_BASE (0x42000000U)

void ENC_init_mdio(MAC_TypeDef *mac_base);
void ENC_wait_for_mdio_idle(MAC_TypeDef *mac_base);
void ENC_write_phy_reg(MAC_TypeDef *mac_base, uint8_t phyaddr, uint8_t regaddr, uint16_t regval);
uint16_t ENC_read_phy_reg(MAC_TypeDef *mac_base, uint8_t phyaddr, uint8_t regaddr);
void ENC_InitializeMemory(uint64_t *addr, uint32_t size);
void ENC_InitEthPhy(void);
void ENC_ReleaseReset(void);

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

void ENC_ReleaseReset(void)
{
    MSS_GPIO_init(GPIO0_LO);
    MSS_GPIO_set_output(GPIO0_LO, MSS_GPIO_12, 1);
    MSS_GPIO_config(GPIO0_LO, MSS_GPIO_12, MSS_GPIO_OUTPUT_MODE);
}

void ENC_InitEthPhy(void)
{
    const uint8_t MDIO_CONTROL = 0x00;
    const uint8_t MDIO_PHY_ID_MSB = 0x02;
    const uint8_t MDIO_PHY_ID_LSB = 0x03;
    const uint8_t MDIO_CONFIG3 = 0x1e;

    const uint32_t DP83867_ID = 0x2000A231;
    const uint16_t CONTROL_POWER_DOWN_MASK = 0x0800;
    const uint16_t CONFIG3_IRQ_EN_MASK = 0x0080;

    int numberOfDetectedEthPhy = 0;
    MAC_TypeDef *mac_base = (MAC_TypeDef*)MSS_MAC1_BASE;
    ENC_init_mdio(mac_base);

    for (uint8_t phyAddr = 0; phyAddr < 32; phyAddr ++)
    {
        // Read ID of PHY and check if it matches DP83867
        uint16_t idMsb = ENC_read_phy_reg(mac_base, phyAddr, MDIO_PHY_ID_MSB);
        uint16_t idLsb = ENC_read_phy_reg(mac_base, phyAddr, MDIO_PHY_ID_LSB);
        uint32_t id = ((uint32_t)idMsb << 16) | idLsb;
        if (id == DP83867_ID)
        {
            mHSS_DEBUG_PRINTF(LOG_NORMAL, "ETH PHY found at MDIO address %i\n", phyAddr);
            numberOfDetectedEthPhy ++;

            // Configure pin INT#/PWDN# for interrupt functionality
            ENC_write_phy_reg(mac_base, phyAddr, MDIO_CONFIG3, CONFIG3_IRQ_EN_MASK);

            // Clear power down flag
            uint16_t reg = ENC_read_phy_reg(mac_base, phyAddr, MDIO_CONTROL);
            ENC_write_phy_reg(mac_base, phyAddr, MDIO_CONTROL, reg & ~CONTROL_POWER_DOWN_MASK);
        }
    }

    if (numberOfDetectedEthPhy == 0) {
        mHSS_DEBUG_PRINTF(LOG_WARN, "No ETH PHY found\n");
    }
}

bool HSS_BoardLateInit(void)
{
    // Make sure peripheral reset is released
    ENC_ReleaseReset();

    // DP83867 requires minimum 200ms between reset and MDIO access
    HSS_SpinDelay_MilliSecs(250);

    ENC_InitEthPhy();

    // With ECC enabled, the DDR memory needs to be initialized to prevent from
    // bus errors caused by reading uninitialized memory
#if LIBERO_SETTING_CFG_ECC_CORRECTION_EN == 1
    ENC_InitializeMemory((uint64_t *)HSS_DDR_GetStart(), HSS_DDR_GetSize());
    ENC_InitializeMemory((uint64_t *)HSS_DDRHi_GetStart(), HSS_DDRHi_GetSize());
#endif

    return true;
}

bool HSS_BoardHandoff(void)
{
    typedef struct
    {
        volatile uint32_t GPIO_CFG[32];
        volatile uint32_t GPIO_IRQ;
        volatile uint32_t GPIO_ALIGN0[3];
        volatile const uint32_t GPIO_IN;
        volatile uint32_t GPIO_ALIGN1[3];
        volatile uint32_t GPIO_OUT;
    } FPGA_GPIO_TypeDef;

    // Switch to uart 1
    mHSS_DEBUG_PRINTF(LOG_NORMAL, "Switching to UART 1 ...\n");
    FPGA_GPIO_TypeDef *gpio_base = (FPGA_GPIO_TypeDef*)UART_SEL_GPIO_BASE;
    gpio_base->GPIO_CFG[0] = 0x5; // Configure to output
    gpio_base->GPIO_OUT = 0x1; // Set output to 1 to select uart 1

    return true;
}
