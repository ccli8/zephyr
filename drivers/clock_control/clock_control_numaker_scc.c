/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_scc

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/logging/log.h>
#include <NuMicro.h>

LOG_MODULE_REGISTER(clock_control_numaker_scc, CONFIG_CLOCK_CONTROL_LOG_LEVEL);

struct numaker_scc_config {
	uint32_t clk_base;
	int hxt;
	int lxt;
	int hirc48;
	uint32_t clk_pclkdiv;
	uint32_t core_clock;
};

#if defined(CONFIG_SOC_SERIES_M2L31X)
static const uint64_t numaker_clkmodidx_tab[] = {
	0x0000000000000001, 0x0000000000000002, 0x0000000000000003, 0x00000000018C0004,
	0x0000000000000007, 0x000000000000000C, 0x000000000000000D, 0x0000000000A01090,
	0x0000000000000018, 0x0000000000000019, 0x000000000000001A, 0x000000000000001B,
	0x000000000000001C, 0x000000000000001D, 0x000000000000001E, 0x000000000000001F,
	0x0000000020000001, 0x0000000025A00002, 0x0000000025B00003, 0x0000000025C00004,
	0x0000000025D00005, 0x0000000026100006, 0x0000000020000007, 0x0000000020000008,
	0x0000000020000009, 0x000000002000000A, 0x000000002000000B, 0x000000002908000C,
	0x000000002990000D, 0x0000000029B0000E, 0x000000002DA0000F, 0x0000000031801110,
	0x0000000031901191, 0x0000000031A11012, 0x0000000031B11093, 0x0000000031C11114,
	0x0000000031D11195, 0x0000000031E11216, 0x0000000031F11297, 0x000000002000001A,
	0x0000000020A0109B, 0x000000002128221C, 0x000000002000001F, 0x000000004DB00006,
	0x0000000040000008, 0x0000000040000009, 0x000000004578000B, 0x000000004000000C,
	0x0000000048800010, 0x0000000048840011, 0x0000000040000016, 0x0000000040000017,
	0x00000000489C0019, 0x000000004000001A, 0x000000004000001B, 0x0000000060000007,
	0x000000006C980008, 0x000000006C9C0009, 0x000000006000000F, 0x0000000080000010,
	0x0000000080000011, 0x0000000081621014, 0x00000000816A1095, 0x0000000081B3101C,
	0x00000000A0000000, 0x00000000A0000001, 0x00000000A0000002, 0x00000000B5600010,
	0x00000000B5080011, 0x00000000A0000012, 0x00000000B5031113, 0x00000000B5A00014,
	0x00000000B5B00015, 0x00000000B5100016, 0x00000000B5180017, 0x00000000B5431218,
	0x00000000A000001B,
};
#endif

static inline int numaker_scc_on(const struct device *dev, clock_control_subsys_t subsys)
{
	ARG_UNUSED(dev);

	struct numaker_scc_subsys *scc_subsys = (struct numaker_scc_subsys *)subsys;

	if (scc_subsys->subsys_id == NUMAKER_SCC_SUBSYS_ID_PCC) {
		SYS_UnlockReg();
#if defined(CONFIG_SOC_SERIES_M2L31X)
		__ASSERT_NO_MSG(scc_subsys->pcc.clk_modidx < ARRAY_SIZE(numaker_clkmodidx_tab));
		CLK_EnableModuleClock(numaker_clkmodidx_tab[scc_subsys->pcc.clk_modidx]);
#else
		CLK_EnableModuleClock(scc_subsys->pcc.clk_modidx);
#endif
		SYS_LockReg();
	} else {
		return -EINVAL;
	}

	return 0;
}

static inline int numaker_scc_off(const struct device *dev, clock_control_subsys_t subsys)
{
	ARG_UNUSED(dev);

	struct numaker_scc_subsys *scc_subsys = (struct numaker_scc_subsys *)subsys;

	if (scc_subsys->subsys_id == NUMAKER_SCC_SUBSYS_ID_PCC) {
		SYS_UnlockReg();
#if defined(CONFIG_SOC_SERIES_M2L31X)
		__ASSERT_NO_MSG(scc_subsys->pcc.clk_modidx < ARRAY_SIZE(numaker_clkmodidx_tab));
		CLK_DisableModuleClock(numaker_clkmodidx_tab[scc_subsys->pcc.clk_modidx]);
#else
		CLK_DisableModuleClock(scc_subsys->pcc.clk_modidx);
#endif
		SYS_LockReg();
	} else {
		return -EINVAL;
	}

	return 0;
}

static inline int numaker_scc_get_rate(const struct device *dev, clock_control_subsys_t subsys,
				       uint32_t *rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(subsys);
	ARG_UNUSED(rate);
	return -ENOTSUP;
}

static inline int numaker_scc_set_rate(const struct device *dev, clock_control_subsys_t subsys,
				       clock_control_subsys_rate_t rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(subsys);
	ARG_UNUSED(rate);
	return -ENOTSUP;
}

static inline int numaker_scc_configure(const struct device *dev, clock_control_subsys_t subsys,
					void *data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(data);

	struct numaker_scc_subsys *scc_subsys = (struct numaker_scc_subsys *)subsys;

	if (scc_subsys->subsys_id == NUMAKER_SCC_SUBSYS_ID_PCC) {
		SYS_UnlockReg();
#if defined(CONFIG_SOC_SERIES_M2L31X)
		__ASSERT_NO_MSG(scc_subsys->pcc.clk_modidx < ARRAY_SIZE(numaker_clkmodidx_tab));
		CLK_SetModuleClock(numaker_clkmodidx_tab[scc_subsys->pcc.clk_modidx],
				   scc_subsys->pcc.clk_src,
				   scc_subsys->pcc.clk_div);
#else
		CLK_SetModuleClock(scc_subsys->pcc.clk_modidx, scc_subsys->pcc.clk_src,
				   scc_subsys->pcc.clk_div);
#endif
		SYS_LockReg();
	} else {
		return -EINVAL;
	}

	return 0;
}

/* System clock controller driver registration */
static struct clock_control_driver_api numaker_scc_api = {
	.on = numaker_scc_on,
	.off = numaker_scc_off,
	.get_rate = numaker_scc_get_rate,
	.set_rate = numaker_scc_set_rate,
	.configure = numaker_scc_configure,
};

/* At most one compatible with status "okay" */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) <= 1,
	     "Requires at most one compatible with status \"okay\"");

#define LOG_OSC_SW(osc, sw)                                                                        \
	if (sw == NUMAKER_SCC_CLKSW_ENABLE) {                                                      \
		LOG_DBG("Enable " #osc);                                                           \
	} else if (sw == NUMAKER_SCC_CLKSW_DISABLE) {                                              \
		LOG_DBG("Disable " #osc);                                                          \
	}

static int numaker_scc_init(const struct device *dev)
{
	const struct numaker_scc_config *cfg = dev->config;

	LOG_DBG("CLK base: 0x%08x", cfg->clk_base);
#if DT_NODE_HAS_PROP(DT_NODELABEL(scc), hxt)
	LOG_OSC_SW(HXT, cfg->hxt);
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(scc), lxt)
	LOG_OSC_SW(LXT, cfg->lxt);
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(scc), hirc48)
	LOG_OSC_SW(HIRC48, cfg->hirc48);
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(scc), clk_pclkdiv)
	LOG_DBG("CLK_PCLKDIV: 0x%08x", cfg->clk_pclkdiv);
#endif
#if DT_NODE_HAS_PROP(DT_NODELABEL(scc), core_clock)
	LOG_DBG("Core clock: %d (Hz)", cfg->core_clock);
#endif

	/*
	 * z_arm_platform_init() will respect above configurations and
	 * actually take charge of system clock control initialization.
	 */

	SystemCoreClockUpdate();
	LOG_DBG("SystemCoreClock: %d (Hz)", SystemCoreClock);

	return 0;
}

#define NUMICRO_SCC_INIT(inst)                                                                     \
	static const struct numaker_scc_config numaker_scc_config_##inst = {                       \
		.clk_base = DT_INST_REG_ADDR(inst),                                                \
		.hxt = DT_INST_ENUM_IDX_OR(inst, hxt, NUMAKER_SCC_CLKSW_UNTOUCHED),                \
		.lxt = DT_INST_ENUM_IDX_OR(inst, lxt, NUMAKER_SCC_CLKSW_UNTOUCHED),                \
		.hirc48 = DT_INST_ENUM_IDX_OR(inst, hirc48, NUMAKER_SCC_CLKSW_UNTOUCHED),          \
		.clk_pclkdiv = DT_INST_PROP_OR(inst, clk_pclkdiv, 0),                              \
		.core_clock = DT_INST_PROP_OR(inst, core_clock, 0),                                \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &numaker_scc_init, NULL, NULL, &numaker_scc_config_##inst,     \
			      PRE_KERNEL_1, CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &numaker_scc_api);

DT_INST_FOREACH_STATUS_OKAY(NUMICRO_SCC_INIT);
