/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __TFM_PLATFORM_HAL_IOCTL_H__
#define __TFM_PLATFORM_HAL_IOCTL_H__

#include <stdint.h>
#include "tfm_platform_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * \brief Helper to define Nuvoton specific Platform IOCTL service function name
 */
#define NVT_TFM_PLAT_IOCTL_SRV(FUNC) NVT_TFM_PLAT_IOCTL_SRV_##FUNC

/*!
 * \brief Helper to define Nuvoton specific Platform IOCTL request code
 */
#define NVT_TFM_PLAT_IOCTL_REQ(FUNC) NVT_TFM_PLAT_IOCTL_REQ_##FUNC

/*!
 * \brief Helper to declare Nuvoton specific Platform IOCTL service function
 */
#define NVT_TFM_PLAT_IOCTL_SRV_DECL(FUNC) NVT_TFM_PLAT_IOCTL_SRV_DECL_(NVT_TFM_PLAT_IOCTL_SRV(FUNC))
#define NVT_TFM_PLAT_IOCTL_SRV_DECL_(FUNC)                                                         \
	enum tfm_platform_err_t FUNC(struct psa_invec *in_vec, struct psa_outvec *out_vec)

/*!
 * \enum nvt_tfm_platform_ioctl_req_t
 *
 * \brief Nuvoton platform specific IOCTL request code
 */
enum nvt_tfm_platform_ioctl_req_t {
	/* BSP SYS driver API */
	NVT_TFM_PLAT_IOCTL_REQ(SYS_LockReg),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_UnlockReg),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_ResetModule_Assert),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_ResetModule_Deassert),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_ResetModule_IsAsserted),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_GPx_MFPx_Read),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_GPx_MFPx_Write),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_GPx_MFOSx_Read),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_GPx_MFOSx_Write),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_USBPHY_Read),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_USBPHY_Write),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_REG_Read),
	NVT_TFM_PLAT_IOCTL_REQ(SYS_REG_Write),

	/* BSP CLK driver API */
	NVT_TFM_PLAT_IOCTL_REQ(CLK_SetModuleClock),
	NVT_TFM_PLAT_IOCTL_REQ(CLK_EnableModuleClock),
	NVT_TFM_PLAT_IOCTL_REQ(CLK_DisableModuleClock),
	NVT_TFM_PLAT_IOCTL_REQ(CLK_REG_Read),
	NVT_TFM_PLAT_IOCTL_REQ(CLK_REG_Write),

	/* Max request code, plays as number of valid request code */
	NVT_TFM_PLAT_IOCTL_REQ(MAX),

	/* Following entry is only to ensure the error code of int32_t size */
	NVT_TFM_PLAT_IOCTL_REQ(INT32_SIZE) = INT32_MAX
};

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_LockReg);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_UnlockReg);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_ResetModule_Assert);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_ResetModule_Deassert);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_ResetModule_IsAsserted);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_GPx_MFPx_Read);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_GPx_MFPx_Write);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_GPx_MFOSx_Read);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_GPx_MFOSx_Write);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_USBPHY_Read);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_USBPHY_Write);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_REG_Read);
NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_REG_Write);
NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_SetModuleClock);
NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_EnableModuleClock);
NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_DisableModuleClock);
NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_REG_Read);
NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_REG_Write);

#ifdef __cplusplus
}
#endif

#endif /* __TFM_PLATFORM_HAL_IOCTL_H__ */
