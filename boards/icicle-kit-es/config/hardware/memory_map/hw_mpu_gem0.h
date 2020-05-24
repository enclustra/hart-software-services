/*******************************************************************************
 * Copyright 2020 Microchip Corporation.
 *
 * SPDX-License-Identifier: MIT
 *
 * @file hw_mpu_gem0.h
 * @author Microchip-FPGA Embedded Systems Solutions
 *
 * Generated using Libero version: 12.900.0.16-PFSOC_MSS:2.0.100
 * Libero design name: ICICLE_MSS
 * MPFS part number used in design: MPFS250T_ES
 * Date generated by Libero: 05-18-2020_08:29:57
 * Format version of XML description: 0.3.1
 * PolarFire SoC Configuration Generator version: 0.3.2
 *
 * Note 1: This file should not be edited. If you need to modify a parameter,
 * without going through the Libero flow or editing the associated xml file,
 * the following method is recommended:
 *   1. edit the file platform//config//software//mpfs_hal//mss_sw_config.h
 *   2. define the value you want to override there. (Note: There is a
 *      commented example in mss_sw_config.h)
 * Note 2: The definition in mss_sw_config.h takes precedence, as
 * mss_sw_config.h is included prior to the hw_mpu_gem0.h in the hal
 * (see platform//mpfs_hal//mss_hal.h)
 *
 */

#ifndef HW_MPU_GEM0_H_
#define HW_MPU_GEM0_H_


#ifdef __cplusplus
extern  "C" {
#endif

#if !defined (LIBERO_SETTING_GEM0_MPU_CFG_PMP0)
/*mpu setup register, 64 bits */
#define LIBERO_SETTING_GEM0_MPU_CFG_PMP0    0x0500000000000022ULL
    /* PMP                               [0:38]  RW value= 0x22 */
    /* RESERVED                          [38:18] RW value= 0x0 */
    /* MODE                              [56:8]  RW value= 0x5 */
#endif
#if !defined (LIBERO_SETTING_GEM0_MPU_CFG_PMP1)
/*mpu setup register, 64 bits */
#define LIBERO_SETTING_GEM0_MPU_CFG_PMP1    0x0500000000000022ULL
    /* PMP                               [0:38]  RW value= 0x22 */
    /* RESERVED                          [38:18] RW value= 0x0 */
    /* MODE                              [56:8]  RW value= 0x5 */
#endif
#if !defined (LIBERO_SETTING_GEM0_MPU_CFG_PMP2)
/*pmp setup register, 64 bits */
#define LIBERO_SETTING_GEM0_MPU_CFG_PMP2    0x0500000000000022ULL
    /* PMP                               [0:38]  RW value= 0x22 */
    /* RESERVED                          [38:18] RW value= 0x0 */
    /* MODE                              [56:8]  RW value= 0x5 */
#endif
#if !defined (LIBERO_SETTING_GEM0_MPU_CFG_PMP3)
/*pmp setup register, 64 bits */
#define LIBERO_SETTING_GEM0_MPU_CFG_PMP3    0x0500000000000022ULL
    /* PMP                               [0:38]  RW value= 0x22 */
    /* RESERVED                          [38:18] RW value= 0x0 */
    /* MODE                              [56:8]  RW value= 0x5 */
#endif
#if !defined (LIBERO_SETTING_GEM0_MPU_CFG_PMP4)
/*pmp setup register, 64 bits */
#define LIBERO_SETTING_GEM0_MPU_CFG_PMP4    0x0500000000000022ULL
    /* PMP                               [0:38]  RW value= 0x22 */
    /* RESERVED                          [38:18] RW value= 0x0 */
    /* MODE                              [56:8]  RW value= 0x5 */
#endif
#if !defined (LIBERO_SETTING_GEM0_MPU_CFG_PMP5)
/*pmp setup register, 64 bits */
#define LIBERO_SETTING_GEM0_MPU_CFG_PMP5    0x0500000000000022ULL
    /* PMP                               [0:38]  RW value= 0x22 */
    /* RESERVED                          [38:18] RW value= 0x0 */
    /* MODE                              [56:8]  RW value= 0x5 */
#endif
#if !defined (LIBERO_SETTING_GEM0_MPU_CFG_PMP6)
/*pmp setup register, 64 bits */
#define LIBERO_SETTING_GEM0_MPU_CFG_PMP6    0x0500000000000022ULL
    /* PMP                               [0:38]  RW value= 0x22 */
    /* RESERVED                          [38:18] RW value= 0x0 */
    /* MODE                              [56:8]  RW value= 0x5 */
#endif
#if !defined (LIBERO_SETTING_GEM0_MPU_CFG_PMP7)
/*pmp setup register, 64 bits */
#define LIBERO_SETTING_GEM0_MPU_CFG_PMP7    0x0500000000000022ULL
    /* PMP                               [0:38]  RW value= 0x22 */
    /* RESERVED                          [38:18] RW value= 0x0 */
    /* MODE                              [56:8]  RW value= 0x5 */
#endif

#ifdef __cplusplus
}
#endif


#endif /* #ifdef HW_MPU_GEM0_H_ */

