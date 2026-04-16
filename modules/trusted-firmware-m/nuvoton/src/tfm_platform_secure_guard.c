/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 Nuvoton Technology Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include "tfm_utils.h"
#include "NuMicro.h"

/* Reserved value for a peripheral's module index not defined
 *
 * Not all peripherals have corresponding SYS/CLK module index.
 * Check sys.h/clk.h under StdDriver for non-used values.
 */
#define NVT_SYS_MODIDX_UNDEF 0xFFFFFFFFUL
#define NVT_CLK_MODIDX_UNDEF 0xFFFFFFFFUL

/* A peripheral's module index, SYS or CLK*/
#define NU_MODCLASS_SYS 0
#define NU_MODCLASS_CLK 1

/* Map for a peripheral's SYS module index non-existent */
#define CACHE_RST NVT_SYS_MODIDX_UNDEF
#define RTC_RST   NVT_SYS_MODIDX_UNDEF
#define WDT1_RST  NVT_SYS_MODIDX_UNDEF

/* Map for a peripheral's CLK module index non-existent */
#define CACHE_MODULE NVT_CLK_MODIDX_UNDEF
#define DAC0_MODULE  DAC_MODULE

/* Shorthand for SCU_INIT_PNSSETx_VAL */
#define NVT_PNSSET(N) SCU_INIT_PNSSET##N##_VAL

/* Shorthand for SCU_INIT_IONSSETx_VAL */
#define NVT_IONSSET(N) SCU_INIT_IONSSET##N##_VAL

/* Helper to define non-secure table entry for peripheral,
 * excluding GPIO
 */
#define NVT_MODIDX_NS_TAB_ENTRY_PERIF(MODNAME, N, n)                                               \
	{MODNAME##_RST, MODNAME##_MODULE, NVT_PNSSET(N) & (1 << n)}

/* Helper to define non-secure table entry for GPIO */
#define NVT_MODIDX_NS_TAB_ENTRY_GP(MODNAME, N)                                                     \
	{NVT_SYS_MODIDX_UNDEF, MODNAME##_MODULE, SCU_INIT_IONSSET_VAL & (1 << N)}

/* Helper to define GPx_MSPx register mask for non-secure  */
#define NVT_GPx_MFPx_MSK_NS(N, M)                                                                  \
	(NVT_GPx_MFPx_MSK_NS_x(N, M, 3) | NVT_GPx_MFPx_MSK_NS_x(N, M, 2) |                         \
	 NVT_GPx_MFPx_MSK_NS_x(N, M, 1) | NVT_GPx_MFPx_MSK_NS_x(N, M, 0))
#define NVT_GPx_MFPx_MSK_NS_x(N, M, n)                                                             \
	((NVT_IONSSET(N) & (1 << (n + 4 * M))) ? (0x1F << (8 * n)) : 0x0)

/* Helper to define GPx_FMOSx register mask for non-secure  */
#define NVT_GPx_MFOS_MSK_NS(N)     NVT_GPx_MFOSx_MSK_NS(N, 0)
#define NVT_GPx_MFOSx_MSK_NS(N, M) (NVT_IONSSET(N) & 0xFFFF)

/* Structure for table entry which keeps a peripheral's security state,
 * indexed by its SYS/CLK module index
 */
typedef struct nvt_modidx_ns_s {
	uint32_t sys_modidx; /* Module index defined in SYS */
	uint32_t clk_modidx; /* Module index defined in CLK */
	uint32_t ns;         /* 0: secure or undefined, 1: non-secure */
} nvt_perif_ns_t;

/* Forward declaration */
static const uint32_t nvt_mfp_msk_ns_tab[];
static const uint32_t nvt_mfos_msk_ns_tab[];
static bool nvt_check_mod_ns(int modclass, uint32_t modidx);

/* Check if SYS module is non-secure */
bool nvt_check_sys_ns(uint32_t modidx)
{
	return nvt_check_mod_ns(NU_MODCLASS_SYS, modidx);
}

/* Check if CLK module is non-secure */
bool nvt_check_clk_ns(uint32_t modidx)
{
	return nvt_check_mod_ns(NU_MODCLASS_CLK, modidx);
}

/* Get MFP mask for non-secure */
uint32_t nvt_get_mfp_mask_ns(uint32_t reg_addr)
{
	uint32_t i;

	i = (uint32_t *)reg_addr - (uint32_t *)&SYS->GPA_MFP0;
	return nvt_mfp_msk_ns_tab[i];
}

/* Get MFOS mask for non-secure */
uint32_t nvt_get_mfos_mask_ns(uint32_t reg_addr)
{
	uint32_t i;

	i = (uint32_t *)reg_addr - (uint32_t *)&SYS->GPA_MFOS;
	return nvt_mfos_msk_ns_tab[i];
}

/* Table whose entry keeps a peripheral's security state, indexed by
 * its SYS/CLK module index
 */
static const nvt_perif_ns_t perif_ns_tab[] = {
#if defined(SCU_INIT_PNSSET0_VAL) && SCU_INIT_PNSSET0_VAL
	/* SCU_INIT_PNSSET0_VAL */
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(PDMA0, 0, 8),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(USBH, 0, 9),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(EBI, 0, 16),
#endif

#if defined(SCU_INIT_PNSSET1_VAL) && SCU_INIT_PNSSET1_VAL
	/* SCU_INIT_PNSSET1_VAL */
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(CANFD0, 1, 0),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(CANFD1, 1, 4),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(CRC, 1, 17),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(CACHE, 1, 22),
#endif

#if defined(SCU_INIT_PNSSET2_VAL) && SCU_INIT_PNSSET2_VAL
	/* SCU_INIT_PNSSET2_VAL */
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(RTC, 2, 1),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(WDT1, 2, 2),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(EADC0, 2, 3),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(ACMP01, 2, 5),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(DAC0, 2, 7),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(EADC1, 2, 11),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(TMR0, 2, 16),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(TMR1, 2, 16),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(TMR2, 2, 17),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(TMR3, 2, 17),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(BPWM0, 2, 26),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(BPWM1, 2, 27),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(PWM0, 2, 28),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(PWM1, 2, 29),
#endif

#if defined(SCU_INIT_PNSSET3_VAL) && SCU_INIT_PNSSET3_VAL
	/* SCU_INIT_PNSSET3_VAL */
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(QSPI0, 3, 0),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(SPI0, 3, 1),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(SPI1, 3, 2),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART0, 3, 16),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART1, 3, 17),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART2, 3, 18),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART3, 3, 19),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART4, 3, 20),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART5, 3, 21),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART6, 3, 22),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART7, 3, 23),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART8, 3, 24),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(UART9, 3, 25),
#endif

#if defined(SCU_INIT_PNSSET4_VAL) && SCU_INIT_PNSSET4_VAL
	/* SCU_INIT_PNSSET4_VAL */
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(I2C0, 4, 0),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(I2C1, 4, 1),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(I2C2, 4, 2),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(I3C0, 4, 6),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(LLSI0, 4, 8),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(LLSI1, 4, 9),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(ELLSI0, 4, 10),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(ELLSI1, 4, 11),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(WWDT1, 4, 24),
#endif

#if defined(SCU_INIT_PNSSET5_VAL) && SCU_INIT_PNSSET5_VAL
	/* SCU_INIT_PNSSET5_VAL */
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(EQEI0, 5, 16),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(EQEI1, 5, 17),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(ECAP0, 5, 20),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(TRNG, 5, 25),
#endif

#if defined(SCU_INIT_PNSSET6_VAL) && SCU_INIT_PNSSET6_VAL
	/* SCU_INIT_PNSSET6_VAL */
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(USBD, 6, 0),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(USCI0, 6, 16),
	NVT_MODIDX_NS_TAB_ENTRY_PERIF(USCI1, 6, 17),
#endif

#if defined(SCU_INIT_IONSSET_VAL) && SCU_INIT_IONSSET_VAL
	/* SCU_INIT_IONSSET_VAL */
	NVT_MODIDX_NS_TAB_ENTRY_GP(GPA, 0),
	NVT_MODIDX_NS_TAB_ENTRY_GP(GPB, 1),
	NVT_MODIDX_NS_TAB_ENTRY_GP(GPC, 2),
	NVT_MODIDX_NS_TAB_ENTRY_GP(GPD, 3),
	NVT_MODIDX_NS_TAB_ENTRY_GP(GPE, 4),
	NVT_MODIDX_NS_TAB_ENTRY_GP(GPF, 5),
	NVT_MODIDX_NS_TAB_ENTRY_GP(GPG, 6),
	NVT_MODIDX_NS_TAB_ENTRY_GP(GPH, 7),
#endif
};

/* Table of GPx_MFPx register mask for non-secure */
static const uint32_t nvt_mfp_msk_ns_tab[] = {
	/* GPA_MFP0/1/2/3 */
	NVT_GPx_MFPx_MSK_NS(0, 0),
	NVT_GPx_MFPx_MSK_NS(0, 1),
	NVT_GPx_MFPx_MSK_NS(0, 2),
	NVT_GPx_MFPx_MSK_NS(0, 3),

	/* GPB_MFP0/1/2/3 */
	NVT_GPx_MFPx_MSK_NS(1, 0),
	NVT_GPx_MFPx_MSK_NS(1, 1),
	NVT_GPx_MFPx_MSK_NS(1, 2),
	NVT_GPx_MFPx_MSK_NS(1, 3),

	/* GPC_MFP0/1/2/3 */
	NVT_GPx_MFPx_MSK_NS(2, 0),
	NVT_GPx_MFPx_MSK_NS(2, 1),
	NVT_GPx_MFPx_MSK_NS(2, 2),
	NVT_GPx_MFPx_MSK_NS(2, 3),

	/* GPD_MFP0/1/2/3 */
	NVT_GPx_MFPx_MSK_NS(3, 0),
	NVT_GPx_MFPx_MSK_NS(3, 1),
	NVT_GPx_MFPx_MSK_NS(3, 2),
	NVT_GPx_MFPx_MSK_NS(3, 3),

	/* GPE_MFP0/1/2/3 */
	NVT_GPx_MFPx_MSK_NS(4, 0),
	NVT_GPx_MFPx_MSK_NS(4, 1),
	NVT_GPx_MFPx_MSK_NS(4, 2),
	NVT_GPx_MFPx_MSK_NS(4, 3),

	/* GPF_MFP0/1/2/3 */
	NVT_GPx_MFPx_MSK_NS(5, 0),
	NVT_GPx_MFPx_MSK_NS(5, 1),
	NVT_GPx_MFPx_MSK_NS(5, 2),
	NVT_GPx_MFPx_MSK_NS(5, 3),

	/* GPG_MFP0/1/2/3 */
	NVT_GPx_MFPx_MSK_NS(6, 0),
	NVT_GPx_MFPx_MSK_NS(6, 1),
	NVT_GPx_MFPx_MSK_NS(6, 2),
	NVT_GPx_MFPx_MSK_NS(6, 3),

	/* GPH_MFP0/1/2/3 */
	NVT_GPx_MFPx_MSK_NS(7, 0),
	NVT_GPx_MFPx_MSK_NS(7, 1),
	NVT_GPx_MFPx_MSK_NS(7, 2),
	NVT_GPx_MFPx_MSK_NS(7, 3),
};

/* Table of GPx_MFOS register mask for non-secure */
static const uint32_t nvt_mfos_msk_ns_tab[] = {
	/* GPA_MFOS */
	NVT_GPx_MFOS_MSK_NS(0),

	/* GPB_MFOS */
	NVT_GPx_MFOS_MSK_NS(1),

	/* GPC_MFOS */
	NVT_GPx_MFOS_MSK_NS(2),

	/* GPD_MFOS */
	NVT_GPx_MFOS_MSK_NS(3),

	/* GPE_MFOS */
	NVT_GPx_MFOS_MSK_NS(4),

	/* GPF_MFOS */
	NVT_GPx_MFOS_MSK_NS(5),

	/* GPG_MFOS */
	NVT_GPx_MFOS_MSK_NS(6),

	/* GPH_MFOS */
	NVT_GPx_MFOS_MSK_NS(7),
};

/* Check if specified module is non-secure
 *
 * \param modclass          Module class (NU_MODCLASS_SYS/NU_MODCLASS_CLK)
 * \param modidx            Module index
 *
 * \return                  0 if not non-secure (secure or undefined), or 1 if non-secure
 *
 * \note                    Undefined module index is treated as not non-secure.
 */
static bool nvt_check_mod_ns(int modclass, uint32_t modidx)
{
	const nvt_perif_ns_t *modidx_ns = perif_ns_tab;
	const nvt_perif_ns_t *modidx_ns_end = perif_ns_tab + ARRAY_SIZE(perif_ns_tab);

	if (modclass == NU_MODCLASS_SYS) {
		for (; modidx_ns != modidx_ns_end; modidx_ns++) {
			if (modidx == modidx_ns->sys_modidx) {
				if (modidx_ns->ns) {
					return true;
				} else {
					return false;
				}
			}
		}
	} else if (modclass == NU_MODCLASS_CLK) {
		for (; modidx_ns != modidx_ns_end; modidx_ns++) {
			if (modidx == modidx_ns->clk_modidx) {
				if (modidx_ns->ns) {
					return true;
				} else {
					return false;
				}
			}
		}
	}

	return false;
}
