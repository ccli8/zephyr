/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "tfm_veneers.h"
#include "psa_manifest/sid.h"

psa_status_t tfm_platform_service(uint32_t ctrl_param, const psa_invec *in_vec,
				  psa_outvec *out_vec);

__attribute__((cmse_nonsecure_entry)) psa_status_t tfm_psa_call_veneer(psa_handle_t handle,
								       uint32_t ctrl_param,
								       const psa_invec *in_vec,
								       psa_outvec *out_vec)
{
	switch (handle) {
	case TFM_PLATFORM_SERVICE_HANDLE:
		return tfm_platform_service(ctrl_param, in_vec, out_vec);
	default:
		return PSA_ERROR_NOT_SUPPORTED;
	};
}
