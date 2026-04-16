/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TFM_PLATFORM_HAL_IOCTL_API_H__
#define __TFM_PLATFORM_HAL_IOCTL_API_H__

#include <stdint.h>
#include "tfm_platform_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief Helper to define Nuvoton specific Platform IOCTL client function name for Non-Secure
 */
#define NVT_TFM_PLAT_IOCTL_NS(FUNC) NVT_TFM_PLAT_IOCTL_NS_##FUNC

void NVT_TFM_PLAT_IOCTL_NS(SYS_LockReg)();
void NVT_TFM_PLAT_IOCTL_NS(SYS_UnlockReg)();
void NVT_TFM_PLAT_IOCTL_NS(SYS_ResetModule_Assert)(uint32_t u32ModuleIndex);
void NVT_TFM_PLAT_IOCTL_NS(SYS_ResetModule_Deassert)(uint32_t u32ModuleIndex);
bool NVT_TFM_PLAT_IOCTL_NS(SYS_ResetModule_IsAsserted)(uint32_t u32ModuleIndex);
uint32_t NVT_TFM_PLAT_IOCTL_NS(SYS_GPx_MFPx_Read)(uint32_t reg);
void NVT_TFM_PLAT_IOCTL_NS(SYS_GPx_MFPx_Write)(uint32_t reg, uint32_t reg_val);
uint32_t NVT_TFM_PLAT_IOCTL_NS(SYS_GPx_MFOSx_Read)(uint32_t reg);
void NVT_TFM_PLAT_IOCTL_NS(SYS_GPx_MFOSx_Write)(uint32_t reg, uint32_t reg_val);
uint32_t NVT_TFM_PLAT_IOCTL_NS(SYS_USBPHY_Read)(uint32_t reg_addr);
void NVT_TFM_PLAT_IOCTL_NS(SYS_USBPHY_Write)(uint32_t reg_addr, uint32_t reg_val);
uint32_t NVT_TFM_PLAT_IOCTL_NS(SYS_REG_Read)(uint32_t reg_addr);
void NVT_TFM_PLAT_IOCTL_NS(SYS_REG_Write)(uint32_t reg_addr, uint32_t reg_val);
void NVT_TFM_PLAT_IOCTL_NS(CLK_SetModuleClock)(uint32_t u32ModuleIndex, uint32_t u32ClkSrc,
					       uint32_t u32ClkDiv);
void NVT_TFM_PLAT_IOCTL_NS(CLK_EnableModuleClock)(uint32_t u32ModuleIndex);
void NVT_TFM_PLAT_IOCTL_NS(CLK_DisableModuleClock)(uint32_t u32ModuleIndex);
uint32_t NVT_TFM_PLAT_IOCTL_NS(CLK_REG_Read)(uint32_t reg_addr);
void NVT_TFM_PLAT_IOCTL_NS(CLK_REG_Write)(uint32_t reg_addr, uint32_t reg_val);

#ifdef __cplusplus
}
#endif

#endif /* __TFM_PLATFORM_HAL_IOCTL_API_H__ */
