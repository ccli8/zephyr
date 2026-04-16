/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#endif
#include "tfm_platform_hal_ioctl.h"
#include "tfm_platform_hal_ioctl_api.h"

#define NVT_TFM_PLAT_IOCTL_CALL(FUNC, INBUF, INSIZE, OUTBUF, OUTSIZE)                              \
	do {                                                                                       \
		enum tfm_platform_err_t rc;                                                        \
                                                                                                   \
		rc = nvt_tfm_plat_ioctl_call(NVT_TFM_PLAT_IOCTL_REQ(FUNC), INBUF, INSIZE, OUTBUF,  \
					     OUTSIZE);                                             \
		if (rc != TFM_PLATFORM_ERR_SUCCESS) {                                              \
			NVT_FATAL_ERROR(FUNC, rc);                                                 \
		}                                                                                  \
	} while (0)

#if defined(__ZEPHYR__)
#define NVT_FATAL_ERROR(FUNC, RC)                                                                  \
	do {                                                                                       \
		printk(#FUNC "() failed (%d)\n", RC);                                              \
		k_oops();                                                                          \
	} while (0)

#else
#define NVT_FATAL_ERROR(FUNC, RC)                                                                  \
	do {                                                                                       \
		printf(#FUNC "() failed (%d)\n", RC);                                              \
		while (1)                                                                          \
			;                                                                          \
	} while (0)
#endif

#if defined(__ZEPHYR__)
#define NVT_GET32_BE(pos)      sys_get_be32(pos)
#define NVT_SET32_BE(pos, val) sys_put_be32(val, pos)
#else
static inline uint32_t NVT_GET32_BE(const uint8_t *pos)
{
	uint32_t val;

	val = *pos++;
	val <<= 8;
	val += *pos++;
	val <<= 8;
	val += *pos++;
	val <<= 8;
	val += *pos;

	return val;
}

static inline void NVT_SET32_BE(uint8_t *pos, uint32_t val)
{
	*pos++ = val >> 24;
	*pos++ = val >> 16;
	*pos++ = val >> 8;
	*pos++ = (val & 0xFF);
}
#endif

static enum tfm_platform_err_t nvt_tfm_plat_ioctl_call(enum nvt_tfm_platform_ioctl_req_t request,
						       const void *inbuf, size_t insize,
						       void *outbuf, size_t outsize)
{
	enum tfm_platform_err_t rc;
	psa_invec invec;
	psa_outvec outvec;

	if (inbuf) {
		invec.base = inbuf;
		invec.len = insize;
		if (outbuf) {
			outvec.base = outbuf;
			outvec.len = outsize;
			rc = tfm_platform_ioctl(request, &invec, &outvec);
		} else {
			rc = tfm_platform_ioctl(request, &invec, NULL);
		}
	} else if (outbuf) {
		outvec.base = outbuf;
		outvec.len = outsize;
		rc = tfm_platform_ioctl(request, NULL, &outvec);
	} else {
		rc = tfm_platform_ioctl(request, NULL, NULL);
	}

	return rc;
}

void NVT_TFM_PLAT_IOCTL_NS(SYS_LockReg)(void)
{
	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_LockReg, NULL, 0, NULL, 0);
}

void NVT_TFM_PLAT_IOCTL_NS(SYS_UnlockReg)(void)
{
	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_UnlockReg, NULL, 0, NULL, 0);
}

void NVT_TFM_PLAT_IOCTL_NS(SYS_ResetModule_Assert)(uint32_t u32ModuleIndex)
{
	uint8_t inbuf[4];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, u32ModuleIndex);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_ResetModule_Assert, inbuf, sizeof(inbuf), NULL, 0);
}

void NVT_TFM_PLAT_IOCTL_NS(SYS_ResetModule_Deassert)(uint32_t u32ModuleIndex)
{
	uint8_t inbuf[4];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, u32ModuleIndex);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_ResetModule_Deassert, inbuf, sizeof(inbuf), NULL, 0);
}

bool NVT_TFM_PLAT_IOCTL_NS(SYS_ResetModule_IsAsserted)(uint32_t u32ModuleIndex)
{
	uint8_t inbuf[4];
	uint8_t outbuf[4];
	bool is_asserted;

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, u32ModuleIndex);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_ResetModule_IsAsserted, inbuf, sizeof(inbuf), outbuf,
				sizeof(outbuf));

	/* Fetch output result */
	is_asserted = !!NVT_GET32_BE(outbuf);

	return is_asserted;
}

uint32_t NVT_TFM_PLAT_IOCTL_NS(SYS_GPx_MFPx_Read)(uint32_t reg_addr)
{
	uint8_t inbuf[4];
	uint8_t outbuf[4];
	int32_t reg_val;

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_GPx_MFPx_Read, inbuf, sizeof(inbuf), outbuf, sizeof(outbuf));

	/* Fetch output result */
	reg_val = NVT_GET32_BE(outbuf);

	return reg_val;
}

void NVT_TFM_PLAT_IOCTL_NS(SYS_GPx_MFPx_Write)(uint32_t reg_addr, uint32_t reg_val)
{
	uint8_t inbuf[8];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);
	NVT_SET32_BE(inbuf + 4, reg_val);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_GPx_MFPx_Write, inbuf, sizeof(inbuf), NULL, 0);
}

uint32_t NVT_TFM_PLAT_IOCTL_NS(SYS_GPx_MFOSx_Read)(uint32_t reg_addr)
{
	uint8_t inbuf[4];
	uint8_t outbuf[4];
	int32_t reg_val;

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_GPx_MFOSx_Read, inbuf, sizeof(inbuf), outbuf, sizeof(outbuf));

	/* Fetch output result */
	reg_val = NVT_GET32_BE(outbuf);

	return reg_val;
}

void NVT_TFM_PLAT_IOCTL_NS(SYS_GPx_MFOSx_Write)(uint32_t reg_addr, uint32_t reg_val)
{
	uint8_t inbuf[8];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);
	NVT_SET32_BE(inbuf + 4, reg_val);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_GPx_MFOSx_Write, inbuf, sizeof(inbuf), NULL, 0);
}

uint32_t NVT_TFM_PLAT_IOCTL_NS(SYS_USBPHY_Read)(uint32_t reg_addr)
{
	uint8_t inbuf[4];
	uint8_t outbuf[4];
	uint32_t reg_val;

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_USBPHY_Read, inbuf, sizeof(inbuf), outbuf, sizeof(outbuf));

	/* Fetch output result */
	reg_val = NVT_GET32_BE(outbuf);

	return reg_val;
}

void NVT_TFM_PLAT_IOCTL_NS(SYS_USBPHY_Write)(uint32_t reg_addr, uint32_t reg_val)
{
	uint8_t inbuf[8];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);
	NVT_SET32_BE(inbuf + 4, reg_val);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_USBPHY_Write, inbuf, sizeof(inbuf), NULL, 0);
}

uint32_t NVT_TFM_PLAT_IOCTL_NS(SYS_REG_Read)(uint32_t reg_addr)
{
	uint8_t inbuf[4];
	uint8_t outbuf[4];
	uint32_t reg_val;

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_REG_Read, inbuf, sizeof(inbuf), outbuf, sizeof(outbuf));

	/* Fetch output result */
	reg_val = NVT_GET32_BE(outbuf);

	return reg_val;
}

void NVT_TFM_PLAT_IOCTL_NS(SYS_REG_Write)(uint32_t reg_addr, uint32_t reg_val)
{
	uint8_t inbuf[8];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);
	NVT_SET32_BE(inbuf + 4, reg_val);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(SYS_REG_Write, inbuf, sizeof(inbuf), NULL, 0);
}

void NVT_TFM_PLAT_IOCTL_NS(CLK_SetModuleClock)(uint32_t u32ModuleIndex, uint32_t u32ClkSrc,
					       uint32_t u32ClkDiv)
{
	uint8_t inbuf[12];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, u32ModuleIndex);
	NVT_SET32_BE(inbuf + 4, u32ClkSrc);
	NVT_SET32_BE(inbuf + 8, u32ClkDiv);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(CLK_SetModuleClock, inbuf, sizeof(inbuf), NULL, 0);
}

void NVT_TFM_PLAT_IOCTL_NS(CLK_EnableModuleClock)(uint32_t u32ModuleIndex)
{
	uint8_t inbuf[4];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, u32ModuleIndex);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(CLK_EnableModuleClock, inbuf, sizeof(inbuf), NULL, 0);
}

void NVT_TFM_PLAT_IOCTL_NS(CLK_DisableModuleClock)(uint32_t u32ModuleIndex)
{
	uint8_t inbuf[4];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, u32ModuleIndex);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(CLK_DisableModuleClock, inbuf, sizeof(inbuf), NULL, 0);
}

uint32_t NVT_TFM_PLAT_IOCTL_NS(CLK_REG_Read)(uint32_t reg_addr)
{
	uint8_t inbuf[4];
	uint8_t outbuf[4];
	uint32_t reg_val;

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(CLK_REG_Read, inbuf, sizeof(inbuf), outbuf, sizeof(outbuf));

	/* Fetch output result */
	reg_val = NVT_GET32_BE(outbuf);

	return reg_val;
}

void NVT_TFM_PLAT_IOCTL_NS(CLK_REG_Write)(uint32_t reg_addr, uint32_t reg_val)
{
	uint8_t inbuf[8];

	/* Set up input parameter for NSC call */
	NVT_SET32_BE(inbuf, reg_addr);
	NVT_SET32_BE(inbuf + 4, reg_val);

	/* Invoke Platform IOCTL function */
	NVT_TFM_PLAT_IOCTL_CALL(CLK_REG_Write, inbuf, sizeof(inbuf), NULL, 0);
}
