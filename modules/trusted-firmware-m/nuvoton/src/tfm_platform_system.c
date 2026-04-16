/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include "tfm_platform_system.h"
#include "tfm_hal_device_header.h"
#include "tfm_platform_hal_ioctl.h"
#include "tfm_utils.h"
#include "NuMicro.h"

void tfm_platform_hal_system_reset(void)
{
	/* Reset the system */
	NVIC_SystemReset();
}

/*!
 * \brief Signature of Nuvoton specific Platform IOCTL service function
 */
typedef enum tfm_platform_err_t (*nvt_plat_ioctl_srv_t)(psa_invec *, psa_outvec *);

/*!
 * \brief Table of Nuvoton specific Platform IOCTL service function
 */
static nvt_plat_ioctl_srv_t nvt_plat_ioctl_srv_tab[] = {
	/* BSP SYS driver API */
	NVT_TFM_PLAT_IOCTL_SRV(SYS_LockReg),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_UnlockReg),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_ResetModule_Assert),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_ResetModule_Deassert),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_ResetModule_IsAsserted),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_GPx_MFPx_Read),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_GPx_MFPx_Write),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_GPx_MFOSx_Read),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_GPx_MFOSx_Write),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_USBPHY_Read),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_USBPHY_Write),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_REG_Read),
	NVT_TFM_PLAT_IOCTL_SRV(SYS_REG_Write),

	/* BSP CLK driver API */
	NVT_TFM_PLAT_IOCTL_SRV(CLK_SetModuleClock),
	NVT_TFM_PLAT_IOCTL_SRV(CLK_EnableModuleClock),
	NVT_TFM_PLAT_IOCTL_SRV(CLK_DisableModuleClock),
	NVT_TFM_PLAT_IOCTL_SRV(CLK_REG_Read),
	NVT_TFM_PLAT_IOCTL_SRV(CLK_REG_Write),
};

_Static_assert(NVT_TFM_PLAT_IOCTL_REQ(MAX) == ARRAY_SIZE(nvt_plat_ioctl_srv_tab),
	       "Platform IOCTL request code/service function tables are not consistent");

enum tfm_platform_err_t tfm_platform_hal_ioctl(tfm_platform_ioctl_req_t request, psa_invec *in_vec,
					       psa_outvec *out_vec)
{
	nvt_plat_ioctl_srv_t srv;

	if (request < 0 || request >= NVT_TFM_PLAT_IOCTL_REQ(MAX)) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	srv = nvt_plat_ioctl_srv_tab[request];

	return srv(in_vec, out_vec);
}
