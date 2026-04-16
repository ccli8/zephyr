/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(__ZEPHYR__)
#include <zephyr/sys/byteorder.h>
#endif
#include "tfm_platform_hal_ioctl.h"
#include "NuMicro.h"

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

#define NVT_TFM_PLAT_IOCTL_CHK_PARAM(INSIZE, OUTSIZE)                                              \
	({                                                                                         \
		bool valid = true;                                                                 \
		if (INSIZE) {                                                                      \
			if (!in_vec || !in_vec->base ||                                            \
			    (in_vec->len != INSIZE && INSIZE != INT_MAX)) {                        \
				valid = false;                                                     \
			}                                                                          \
		} else {                                                                           \
			if (in_vec) {                                                              \
				valid = false;                                                     \
			}                                                                          \
		}                                                                                  \
		if (OUTSIZE) {                                                                     \
			if (!out_vec || !out_vec->base ||                                          \
			    (out_vec->len != OUTSIZE && OUTSIZE != INT_MAX)) {                     \
				valid = false;                                                     \
			}                                                                          \
		} else {                                                                           \
			if (out_vec) {                                                             \
				valid = false;                                                     \
			}                                                                          \
		}                                                                                  \
		valid;                                                                             \
	})

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_LockReg)
{
	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(0, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	SYS_LockReg();

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_UnlockReg)
{
	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(0, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	SYS_UnlockReg();

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_ResetModule_Assert)
{
	uint32_t u32ModuleIndex;
	uint32_t u32tmpVal;
	uint32_t u32tmpAddr;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	u32ModuleIndex = NVT_GET32_BE(in_vec->base);

	/* Generate reset signal to the corresponding module */
	u32tmpVal = (1UL << (u32ModuleIndex & 0x0000001fUL));
	u32tmpAddr = (uint32_t)&SYS->IPRST0 + ((u32ModuleIndex >> 24UL));
	*(volatile uint32_t *)u32tmpAddr |= u32tmpVal;

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_ResetModule_Deassert)
{
	uint32_t u32ModuleIndex;
	uint32_t u32tmpVal;
	uint32_t u32tmpAddr;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	u32ModuleIndex = NVT_GET32_BE(in_vec->base);

	/* Release corresponding module from reset state */
	u32tmpVal = (1UL << (u32ModuleIndex & 0x0000001fUL));
	u32tmpAddr = (uint32_t)&SYS->IPRST0 + ((u32ModuleIndex >> 24UL));
	*(volatile uint32_t *)u32tmpAddr &= ~u32tmpVal;

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_ResetModule_IsAsserted)
{
	uint32_t u32ModuleIndex;
	uint32_t u32tmpVal;
	uint32_t u32tmpAddr;
	uint32_t u32Asserted;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 4)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	u32ModuleIndex = NVT_GET32_BE(in_vec->base);

	/* Test reset state of the corresponding module */
	u32tmpVal = (1UL << (u32ModuleIndex & 0x0000001fUL));
	u32tmpAddr = (uint32_t)&SYS->IPRST0 + ((u32ModuleIndex >> 24UL));
	u32Asserted = (*(volatile uint32_t *)u32tmpAddr) & u32tmpVal;

	NVT_SET32_BE(out_vec->base, u32Asserted);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_GPx_MFPx_Read)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 4)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = *((volatile uint32_t *)reg_addr);

	/* Check register address validity */
	if (reg_addr < (uint32_t)&SYS->GPA_MFP0 || reg_addr > (uint32_t)&SYS->GPH_MFP2) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	NVT_SET32_BE(out_vec->base, reg_val);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_GPx_MFPx_Write)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(8, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = NVT_GET32_BE(((const uint8_t *)in_vec->base) + 4);

	/* Check register address validity */
	if (reg_addr < (uint32_t)&SYS->GPA_MFP0 || reg_addr > (uint32_t)&SYS->GPH_MFP2) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	*((volatile uint32_t *)reg_addr) = reg_val;

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_GPx_MFOSx_Read)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 4)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = *((volatile uint32_t *)reg_addr);

	/* Check register address validity */
	if (reg_addr < (uint32_t)&SYS->GPA_MFOS || reg_addr > (uint32_t)&SYS->GPH_MFOS) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	NVT_SET32_BE(out_vec->base, reg_val);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_GPx_MFOSx_Write)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(8, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = NVT_GET32_BE(((const uint8_t *)in_vec->base) + 4);

	/* Check register address validity */
	if (reg_addr < (uint32_t)&SYS->GPA_MFOS || reg_addr > (uint32_t)&SYS->GPH_MFOS) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	*((volatile uint32_t *)reg_addr) = reg_val;

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_USBPHY_Read)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 4)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = *((volatile uint32_t *)reg_addr);

	/* Check register address validity */
	if (reg_addr != (uint32_t)&SYS->USBPHY) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	NVT_SET32_BE(out_vec->base, reg_val);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_USBPHY_Write)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(8, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = NVT_GET32_BE(((const uint8_t *)in_vec->base) + 4);

	/* Check register address validity */
	if (reg_addr != (uint32_t)&SYS->USBPHY) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	*((volatile uint32_t *)reg_addr) = reg_val;

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_REG_Read)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 4)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = *((volatile uint32_t *)reg_addr);

	/* Check register address validity */
	if ((reg_addr < (uint32_t)SYS) || (reg_addr >= (uint32_t)(SYS + 1))) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	NVT_SET32_BE(out_vec->base, reg_val);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(SYS_REG_Write)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(8, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = NVT_GET32_BE(((const uint8_t *)in_vec->base) + 4);

	/* Check register address validity */
	if ((reg_addr < (uint32_t)SYS) || (reg_addr >= (uint32_t)(SYS + 1))) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	*((volatile uint32_t *)reg_addr) = reg_val;

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_SetModuleClock)
{
	uint32_t u32ModuleIndex;
	uint32_t u32ClkSrc;
	uint32_t u32ClkDiv;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(12, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	u32ModuleIndex = NVT_GET32_BE(in_vec->base);
	u32ClkSrc = NVT_GET32_BE(((const uint8_t *)in_vec->base) + 4);
	u32ClkDiv = NVT_GET32_BE(((const uint8_t *)in_vec->base) + 8);

	CLK_SetModuleClock(u32ModuleIndex, u32ClkSrc, u32ClkDiv);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_EnableModuleClock)
{
	uint32_t u32ModuleIndex;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	u32ModuleIndex = NVT_GET32_BE(in_vec->base);

	CLK_EnableModuleClock(u32ModuleIndex);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_DisableModuleClock)
{
	uint32_t u32ModuleIndex;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	u32ModuleIndex = NVT_GET32_BE(in_vec->base);

	CLK_DisableModuleClock(u32ModuleIndex);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_REG_Read)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(4, 4)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = *((volatile uint32_t *)reg_addr);

	/* Check register address validity */
	if ((reg_addr < (uint32_t)CLK) || (reg_addr >= (uint32_t)(CLK + 1))) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	NVT_SET32_BE(out_vec->base, reg_val);

	return TFM_PLATFORM_ERR_SUCCESS;
}

NVT_TFM_PLAT_IOCTL_SRV_DECL(CLK_REG_Write)
{
	uint32_t reg_addr;
	uint32_t reg_val;

	/* Check parameter validity */
	if (!NVT_TFM_PLAT_IOCTL_CHK_PARAM(8, 0)) {
		return TFM_PLATFORM_ERR_INVALID_PARAM;
	}

	reg_addr = NVT_GET32_BE(in_vec->base);
	reg_val = NVT_GET32_BE(((const uint8_t *)in_vec->base) + 4);

	/* Check register address validity */
	if ((reg_addr < (uint32_t)CLK) || (reg_addr >= (uint32_t)(CLK + 1))) {
		return TFM_PLATFORM_ERR_NOT_SUPPORTED;
	}

	*((volatile uint32_t *)reg_addr) = reg_val;

	return TFM_PLATFORM_ERR_SUCCESS;
}
