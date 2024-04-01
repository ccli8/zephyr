/*
 * Copyright (c) 2024 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_vbus

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb_c/usbc_vbus.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(vbus_numaker, CONFIG_USBC_LOG_LEVEL);

#include <soc.h>
#include <NuMicro.h>

/* Implementation notes on NuMaker TCPC/PPC/VBUS
 *
 * PPC and VBUS rely on TCPC/UTCPD and are just pseudo. They are completely
 * implemented in TCPC/UTCPD.
 */

/* Immutable device context */
struct numaker_vbus_config {
	const struct device *tcpc_dev;
};

/* Mutable device context */
struct numaker_vbus_data {
	uint32_t dummy;
};

/* TCPC exported */
bool numaker_tcpc_vbus_check_level(const struct device *dev, enum tc_vbus_level level);
int numaker_tcpc_vbus_measure(const struct device *dev, int *vbus_meas);
int numaker_tcpc_vbus_discharge(const struct device *dev, bool enable);
int numaker_tcpc_vbus_enable(const struct device *dev, bool enable);

static int numaker_vbus_init(const struct device *dev)
{
	const struct numaker_vbus_config *const config = dev->config;
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

static bool numaker_vbus_check_level(const struct device *dev, enum tc_vbus_level level)
{
	const struct numaker_vbus_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_vbus_check_level(tcpc_dev, level);
}

static int numaker_vbus_measure(const struct device *dev, int *vbus_meas)
{
	const struct numaker_vbus_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_vbus_measure(tcpc_dev, vbus_meas);
}

static int numaker_vbus_discharge(const struct device *dev, bool enable)
{
	const struct numaker_vbus_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_vbus_discharge(tcpc_dev, enable);
}

static int numaker_vbus_enable(const struct device *dev, bool enable)
{
	const struct numaker_vbus_config *const config = dev->config;
	const struct device *tcpc_dev = config->tcpc_dev;

	return numaker_tcpc_vbus_enable(tcpc_dev, enable);
}

static const struct usbc_vbus_driver_api numaker_vbus_driver_api = {
	.check_level = numaker_vbus_check_level,
	.measure = numaker_vbus_measure,
	.discharge = numaker_vbus_discharge,
	.enable = numaker_vbus_enable,
};

#define NUMAKER_TCPC(inst) DT_INST_PARENT(inst)

#define VBUS_NUMAKER_INIT(inst)                                                                    \
	static const struct numaker_vbus_config numaker_vbus_config_##inst = {                     \
		.tcpc_dev = DEVICE_DT_GET(NUMAKER_TCPC(inst)),                                     \
	};                                                                                         \
                                                                                                   \
	static struct numaker_vbus_data numaker_vbus_data_##inst;                                  \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, numaker_vbus_init, NULL, &numaker_vbus_data_##inst,            \
			      &numaker_vbus_config_##inst, POST_KERNEL,                            \
			      CONFIG_USBC_VBUS_INIT_PRIORITY, &numaker_vbus_driver_api);

DT_INST_FOREACH_STATUS_OKAY(VBUS_NUMAKER_INIT);
