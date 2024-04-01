/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_ppc

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb_c/usbc_ppc.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ppc_numaker, CONFIG_USBC_LOG_LEVEL);

#include <soc.h>
#include <NuMicro.h>

/* Implementation notes on NuMaker TCPC/PPC/VBUS
 *
 * PPC and VBUS rely on TCPC/UTCPD and are just pseudo. They are completely
 * implemented in TCPC/UTCPD.
 */

/* Immutable device context */
struct numaker_ppc_config {
	const struct device *tcpc_dev;
};

/* Mutable device context */
struct numaker_ppc_data {
	uint32_t dummy;
};

/* TCPC exported */
int numaker_tcpc_ppc_is_dead_battery_mode(const struct device *dev);
int numaker_tcpc_ppc_exit_dead_battery_mode(const struct device *dev);
int numaker_tcpc_ppc_is_vbus_source(const struct device *dev);
int numaker_tcpc_ppc_is_vbus_sink(const struct device *dev);
int numaker_tcpc_ppc_set_snk_ctrl(const struct device *dev, bool enable);
int numaker_tcpc_ppc_set_src_ctrl(const struct device *dev, bool enable);
int numaker_tcpc_ppc_set_vbus_discharge(const struct device *dev, bool enable);
int numaker_tcpc_ppc_is_vbus_present(const struct device *dev);
int numaker_tcpc_ppc_set_event_handler(const struct device *dev, usbc_ppc_event_cb_t handler,
				       void *data);
int numaker_tcpc_ppc_dump_regs(const struct device *dev);

static int numaker_ppc_init(const struct device *dev)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;
	int rc = 0;

	/* Rely on TCPC */
	if (!device_is_ready(tcpc_dev)) {
		LOG_ERR("TCPC device not ready");
		rc = -ENODEV;
		goto cleanup;
	}

cleanup:

	return rc;
}

static int numaker_ppc_is_dead_battery_mode(const struct device *dev)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_is_dead_battery_mode(tcpc_dev);
}

static int numaker_ppc_exit_dead_battery_mode(const struct device *dev)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_exit_dead_battery_mode(tcpc_dev);
}

static int numaker_ppc_is_vbus_source(const struct device *dev)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_is_vbus_source(tcpc_dev);
}

static int numaker_ppc_is_vbus_sink(const struct device *dev)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_is_vbus_sink(tcpc_dev);
}

static int numaker_ppc_set_snk_ctrl(const struct device *dev, bool enable)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_set_snk_ctrl(tcpc_dev, enable);
}

static int numaker_ppc_set_src_ctrl(const struct device *dev, bool enable)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_set_src_ctrl(tcpc_dev, enable);
}

static int numaker_ppc_set_vbus_discharge(const struct device *dev, bool enable)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_set_vbus_discharge(tcpc_dev, enable);
}

static int numaker_ppc_is_vbus_present(const struct device *dev)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_is_vbus_present(tcpc_dev);
}

static int numaker_ppc_set_event_handler(const struct device *dev, usbc_ppc_event_cb_t handler,
					 void *data)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_set_event_handler(tcpc_dev, handler, data);
}

static int numaker_ppc_dump_regs(const struct device *dev)
{
	const struct numaker_ppc_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_ppc_dump_regs(tcpc_dev);
}

static const struct usbc_ppc_driver_api numaker_ppc_driver_api = {
	.is_dead_battery_mode = numaker_ppc_is_dead_battery_mode,
	.exit_dead_battery_mode = numaker_ppc_exit_dead_battery_mode,
	.is_vbus_source = numaker_ppc_is_vbus_source,
	.is_vbus_sink = numaker_ppc_is_vbus_sink,
	.set_snk_ctrl = numaker_ppc_set_snk_ctrl,
	.set_src_ctrl = numaker_ppc_set_src_ctrl,
	.set_vbus_discharge = numaker_ppc_set_vbus_discharge,
	.is_vbus_present = numaker_ppc_is_vbus_present,
	.set_event_handler = numaker_ppc_set_event_handler,
	.dump_regs = numaker_ppc_dump_regs,
};

#define NUMAKER_TCPC(inst) DT_INST_PARENT(inst)

#define PPC_NUMAKER_INIT(inst)                                                                     \
	static const struct numaker_ppc_config numaker_ppc_config_##inst = {                       \
		.tcpc_dev = DEVICE_DT_GET(NUMAKER_TCPC(inst)),                                     \
	};                                                                                         \
                                                                                                   \
	static struct numaker_ppc_data numaker_ppc_data_##inst;                                    \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, numaker_ppc_init, NULL, &numaker_ppc_data_##inst,              \
			      &numaker_ppc_config_##inst, POST_KERNEL,                             \
			      CONFIG_USBC_PPC_INIT_PRIORITY, &numaker_ppc_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PPC_NUMAKER_INIT);
