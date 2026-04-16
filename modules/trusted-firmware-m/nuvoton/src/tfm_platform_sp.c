/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/byteorder.h>
#include "tfm_platform_system.h"
#include "tfm_psa_call_pack.h"

psa_status_t tfm_platform_service(uint32_t ctrl_param, const psa_invec *in_vec, psa_outvec *out_vec)
{
	int32_t type = PARAM_UNPACK_TYPE(ctrl_param);
	size_t in_len = PARAM_UNPACK_IN_LEN(ctrl_param);
	size_t out_len = PARAM_UNPACK_OUT_LEN(ctrl_param);
	tfm_platform_ioctl_req_t request;

	switch (type) {
	case TFM_PLATFORM_API_ID_SYSTEM_RESET:
		tfm_platform_hal_system_reset();
		return PSA_SUCCESS;
	case TFM_PLATFORM_API_ID_IOCTL:
		if (in_vec == NULL || in_len < 1) {
			return PSA_ERROR_PROGRAMMER_ERROR;
		}
		if (in_vec->base == NULL || in_vec->len != sizeof(request)) {
			return PSA_ERROR_PROGRAMMER_ERROR;
		}
		if (out_vec == NULL && out_len != 0) {
			return PSA_ERROR_PROGRAMMER_ERROR;
		}
		if (out_vec != NULL && out_len == 0) {
			return PSA_ERROR_PROGRAMMER_ERROR;
		}
#if defined(CONFIG_LITTLE_ENDIAN)
		request = (tfm_platform_ioctl_req_t)sys_get_le32(in_vec->base);
#else
		request = (tfm_platform_ioctl_req_t)sys_get_be32(in_vec->base);
#endif
		if (in_len < 2) {
			return (psa_status_t)tfm_platform_hal_ioctl(request, NULL, out_vec);
		} else {
			return (psa_status_t)tfm_platform_hal_ioctl(
				request, (psa_invec *)in_vec + 1, out_vec);
		}
	default:
		return PSA_ERROR_NOT_SUPPORTED;
	}
}
