/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_numaker_usbd

#include <zephyr/kernel.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/sys/math_extras.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/clock_control_numaker.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(udc_numaker, CONFIG_UDC_DRIVER_LOG_LEVEL);

#include <soc.h>
#include <NuMicro.h>

#include "udc_common.h"

/* USBD notes
 *
 * 1. Require 48MHz clock source
 *    (1) Not support HIRC48 as clock source. It involves trim with USB SOF packets
 *        and isn't suitable in HAL.
 *    (2) Instead of HICR48, core clock is required to be multiple of 48MHz e.g. 192MHz,
 *        to generate necessary 48MHz.
 */

/* For bus reset, keep 'SE0' (USB spec: SE0 >= 2.5 ms) */
#define NUMAKER_USBD_BUS_RESET_DRV_SE0_US 3000

/* For bus resume, generate 'K' (USB spec: 'K' >= 1 ms) */
#define NUMAKER_USBD_BUS_RESUME_DRV_K_US 1500

/* Reserve DMA buffer for Setup/CTRL OUT/CTRL IN, required to be 8-byte aligned */
#define NUMAKER_USBD_DMABUF_SIZE_SETUP   8
#define NUMAKER_USBD_DMABUF_SIZE_CTRLOUT 64
#define NUMAKER_USBD_DMABUF_SIZE_CTRLIN  64

enum numaker_usbd_msg_type {
	/* Bus UDC event */
	NUMAKER_USBD_MSG_TYPE_UDC_EVENT,
	/* Setup packet received */
	NUMAKER_USBD_MSG_TYPE_SETUP,
	/* OUT transaction for specific EP completed */
	NUMAKER_USBD_MSG_TYPE_DOUT,
	/* IN transaction for specific EP completed */
	NUMAKER_USBD_MSG_TYPE_DIN,
	/* Resume queued transfer for specific EP */
	NUMAKER_USBD_MSG_TYPE_XFER,
	/* S/W reconnect */
	NUMAKER_USBD_MSG_TYPE_SW_RECONN,
};

struct numaker_usbd_msg {
	numaker_usbd_msg_type type;
	union {
		struct {
			enum udc_event_type type;
		} udc_event;
		struct {
			uint8_t packet[8];
		} setup;
		struct {
			uint8_t ep;
		} dout;
		struct {
			uint8_t ep;
		} din;
		struct {
			uint8_t ep;
		} xfer;
	};
};

/* EP H/W context */
struct numaker_usbd_ep {
	bool valid;

	bool nak_clr; /* NAK cleared (ACK next transaction) */

	const struct device *dev; /* Pointer to the containing device */

	uint8_t ep_hw_idx;  /* BSP USBD driver EP index EP0, EP1, EP2, etc */
	uint32_t ep_hw_cfg; /* BSP USBD driver EP configuration */

	/* EP DMA buffer */
	bool dmabuf_valid;
	uint32_t dmabuf_base;
	uint32_t dmabuf_size;

	/* On USBD, no H/W FIFO. Simulate based on above DMA buffer with
	 * one-shot implementation
	 */
	uint32_t read_fifo_pos;
	uint32_t read_fifo_used;
	uint32_t write_fifo_pos;
	uint32_t write_fifo_free;

	/* NOTE: On USBD, Setup and CTRL OUT are not completely separated. CTRL OUT MXPLD
	 * can be overridden to 8 by next Setup. To overcome it, we make one copy of CTRL
	 * OUT MXPLD immediately on its interrupt.
	 */
	uint32_t mxpld_ctrlout;

	/* EP address */
	bool addr_valid;
	uint8_t addr;

	/* EP MPS */
	bool mps_valid;
	uint16_t mps;

	usb_dc_ep_callback cb; /* EP callback function */
};

/* Immutable device context */
struct udc_numaker_config {
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	uint32_t ep_cfg_in_size;
	uint32_t ep_cfg_out_size;
	USBD_T *base;
	const struct reset_dt_spec reset;
	uint32_t clk_modidx;
	uint32_t clk_src;
	uint32_t clk_div;
	const struct device *clkctrl_dev;
	void (*irq_config_func)(const struct device *dev);
	void (*irq_unconfig_func)(const struct device *dev);
	const struct pinctrl_dev_config *pincfg;
	uint32_t dmabuf_size;
	bool disallow_iso_inout_same;
	void (*make_thread)(const struct device *dev);
};

/* EP H/W context manager */
struct numaker_usbd_ep_mgmt {
	/* EP H/W context management
	 *
	 * Allocate-only, and de-allocate all on re-initialize in usb_dc_attach().
	 */
	uint8_t ep_idx;

	/* DMA buffer management
	 *
	 * Allocate-only, and de-allocate all on re-initialize in usb_dc_attach().
	 */
	uint32_t dmabuf_pos;
};

/* Mutable device context */
struct udc_numaker_data {
	uint8_t addr; /* Host assigned USB device address */

	struct k_msgq *msgq;

	usb_dc_status_callback status_cb; /* Status callback function */

	struct numaker_usbd_ep_mgmt ep_mgmt; /* EP management */

	struct numaker_usbd_ep *ep_pool;
	uint32_t ep_pool_size;

	struct k_thread thread_data;	
};

static inline void numaker_usbd_sw_connect(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	USBD_T *const base = config->base;

	/* Clear all interrupts first for clean */
	base->INTSTS = base->INTSTS;

	/* Enable relevant interrupts */
	base->INTEN = USBD_INT_BUS | USBD_INT_USB | USBD_INT_FLDET | USBD_INT_WAKEUP | USBD_INT_SOF;

	/* Clear SE0 for connect */
	base->SE0 &= ~USBD_DRVSE0;
}

static inline void numaker_usbd_sw_disconnect(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	USBD_T *const base = config->base;

	/* Set SE0 for disconnect */
	base->SE0 |= USBD_DRVSE0;
}

static inline void numaker_usbd_sw_reconnect(const struct device *dev)
{
	/* Keep SE0 to trigger bus reset */
	numaker_usbd_sw_disconnect(dev);
	k_sleep(K_USEC(NUMAKER_USBD_BUS_RESET_DRV_SE0_US));
	numaker_usbd_sw_connect(dev);
}

static inline void numaker_usbd_reset_addr(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	USBD_T *const base = config->base;

	base->FADDR = 0;
	priv->addr = 0;
}

static inline void numaker_usbd_set_addr(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	USBD_T *const base = config->base;

	if (base->FADDR != priv->addr) {
		base->FADDR = priv->addr;
	}
}

/* USBD EP base by e.g. EP0, EP1, ... */
static inline USBD_EP_T *numaker_usbd_ep_base(const struct device *dev, uint32_t ep_hw_idx)
{
	const struct udc_numaker_config *config = dev->config;
	USBD_T *const base = config->base;

	return base->EP + ep_hw_idx;
}

static inline uint32_t udc_numaker_ep_fifo_max(struct numaker_usbd_ep *ep_cur)
{
	/* NOTE: For one-shot implementation, effective size of EP FIFO is limited to EP MPS */
	__ASSERT_NO_MSG(ep_cur->dmabuf_valid);
	__ASSERT_NO_MSG(ep_cur->mps_valid);
	__ASSERT_NO_MSG(ep_cur->mps <= ep_cur->dmabuf_size);
	return ep_cur->mps;
}

static inline uint32_t udc_numaker_ep_fifo_used(struct numaker_usbd_ep *ep_cur)
{
	__ASSERT_NO_MSG(ep_cur->dmabuf_valid);

	return USB_EP_DIR_IS_OUT(ep_cur->addr)
		       ? ep_cur->read_fifo_used
		       : udc_numaker_ep_fifo_max(ep_cur) - ep_cur->write_fifo_free;
}

/* Reset EP FIFO
 *
 * NOTE: EP FIFO is based on EP DMA buffer, which may not be configured yet.
 */
static void udc_numaker_ep_fifo_reset(struct numaker_usbd_ep *ep_cur)
{
	if (ep_cur->dmabuf_valid && ep_cur->mps_valid) {
		if (USB_EP_DIR_IS_OUT(ep_cur->addr)) {
			/* Read FIFO */
			ep_cur->read_fifo_pos = ep_cur->dmabuf_base;
			ep_cur->read_fifo_used = 0;
		} else {
			/* Write FIFO */
			ep_cur->write_fifo_pos = ep_cur->dmabuf_base;
			ep_cur->write_fifo_free = udc_numaker_ep_fifo_max(ep_cur);
		}
	}
}

static inline void numaker_usbd_ep_set_stall(struct numaker_usbd_ep *ep_cur)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	/* Set EP to stalled */
	ep_base->CFGP |= USBD_CFGP_SSTALL_Msk;
}

/* Reset EP to unstalled and data toggle bit to 0 */
static inline void numaker_usbd_ep_clear_stall_n_data_toggle(struct numaker_usbd_ep *ep_cur)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	/* Reset EP to unstalled */
	ep_base->CFGP &= ~USBD_CFGP_SSTALL_Msk;

	/* Reset EP data toggle bit to 0 */
	ep_base->CFG &= ~USBD_CFG_DSQSYNC_Msk;
}

static int numaker_usbd_send_msg(const struct device *dev, const struct numaker_usbd_msg *msg)
{
	struct udc_numaker_data *priv = udc_get_private(dev);
	int err;

	err = k_msgq_put(priv->msgq, msg, K_NO_WAIT);
	if (err < 0) {
		/* Try to recover by S/W reconnect */
		struct numaker_usbd_msg msg_reconn = {
			.type = NUMAKER_USBD_MSG_TYPE_SW_RECONN,
		};

		LOG_ERR("Message queue overflow");

		/* Discard all not yet received messages for error recovery below */
		k_msgq_purge(priv->msgq);

		err = k_msgq_put(priv->msgq, &msg_reconn, K_NO_WAIT);
		if (err < 0) {
			LOG_ERR("Message queue overflow again");
		}
	}

	return err;
}

static int numaker_usbd_hw_setup(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	USBD_T *const base = config->base;
	int err;
	struct numaker_scc_subsys scc_subsys;

	/* Reset controller ready? */
	if (!device_is_ready(config->reset.dev)) {
		LOG_ERR("Reset controller not ready");
		return -ENODEV;
	}

	SYS_UnlockReg();

	/* Configure USB PHY for USBD */
	SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) |
		      (SYS_USBPHY_USBROLE_STD_USBD | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk);

	/* Invoke Clock controller to enable module clock */
	memset(&scc_subsys, 0x00, sizeof(scc_subsys));
	scc_subsys.subsys_id = NUMAKER_SCC_SUBSYS_ID_PCC;
	scc_subsys.pcc.clk_modidx = config->clk_modidx;
	scc_subsys.pcc.clk_src = config->clk_src;
	scc_subsys.pcc.clk_div = config->clk_div;

	/* Equivalent to CLK_EnableModuleClock() */
	err = clock_control_on(config->clkctrl_dev, (clock_control_subsys_t)&scc_subsys);
	if (err < 0) {
		goto cleanup;
	}
	/* Equivalent to CLK_SetModuleClock() */
	err = clock_control_configure(config->clkctrl_dev, (clock_control_subsys_t)&scc_subsys,
				     NULL);
	if (err < 0) {
		goto cleanup;
	}

	/* Configure pinmux (NuMaker's SYS MFP) */
	err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		goto cleanup;
	}

	/* Invoke Reset controller to reset module to default state */
	/* Equivalent to SYS_ResetModule()
	 */
	reset_line_toggle_dt(&config->reset);

	/* Initialize USBD engine */
	/* NOTE: BSP USBD driver: ATTR = 0x7D0 */
	base->ATTR = USBD_ATTR_BYTEM_Msk | BIT(9) | USBD_ATTR_DPPUEN_Msk | USBD_ATTR_USBEN_Msk |
		     BIT(6) | USBD_ATTR_PHYEN_Msk;

	/* Set SE0 for S/W disconnect */
	numaker_usbd_sw_disconnect(dev);

	/* NOTE: Ignore DT maximum-speed with USBD fixed to full-speed */

	/* Initialize IRQ */
	config->irq_config_func(dev);

cleanup:

	SYS_LockReg();

	return err;
}

static void numaker_usbd_hw_shutdown(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	USBD_T *const base = config->base;
	struct numaker_scc_subsys scc_subsys;

	SYS_UnlockReg();

	/* Uninitialize IRQ */
	config->irq_unconfig_func(dev);

	/* Set SE0 for S/W disconnect */
	numaker_usbd_sw_disconnect(dev);

	/* Disable USB PHY */
	base->ATTR &= ~USBD_PHY_EN;

	/* Invoke Clock controller to disable module clock */
	memset(&scc_subsys, 0x00, sizeof(scc_subsys));
	scc_subsys.subsys_id = NUMAKER_SCC_SUBSYS_ID_PCC;
	scc_subsys.pcc.clk_modidx = config->clk_modidx;

	/* Equivalent to CLK_DisableModuleClock() */
	clock_control_off(config->clkctrl_dev, (clock_control_subsys_t)&scc_subsys);

	/* Invoke Reset controller to reset module to default state */
	/* Equivalent to SYS_ResetModule() */
	reset_line_toggle_dt(&config->reset);

	SYS_LockReg();
}

/* Interrupt top half processing for bus reset */
static void numaker_usbd_bus_reset_th(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	USBD_EP_T *ep_base;

	for (uint32_t i = 0ul; i < priv->ep_pool_size; i++) {
		ep_base = numaker_usbd_ep_base(dev, EP0 + i);

		/* Cancel EP on-going transaction */
		ep_base->CFGP |= USBD_CFGP_CLRRDY_Msk;

		/* Reset EP to unstalled */
		ep_base->CFGP &= ~USBD_CFGP_SSTALL_Msk;

		/* Reset EP data toggle bit to 0 */
		ep_base->CFG &= ~USBD_CFG_DSQSYNC_Msk;

		/* Except EP0/EP1 kept resident for CTRL OUT/IN, disable all other EPs */
		if (i >= 2) {
			ep_base->CFG = 0;
		}
	}

	numaker_usbd_reset_addr(dev);
}

/* USBD SRAM base for DMA */
static inline uint32_t numaker_usbd_buf_base(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	USBD_T *const base = config->base;

	return ((uint32_t)base + 0x800ul);
}

/* Copy Setup packet to user buffer */
static void numaker_usbd_setup_copy_to_user(const struct device *dev, uint8_t *usrbuf)
{
	const struct udc_numaker_config *config = dev->config;
	USBD_T *const base = config->base;
	uint32_t dmabuf_addr;

	dmabuf_addr = numaker_usbd_buf_base(dev) + (base->STBUFSEG & USBD_STBUFSEG_STBUFSEG_Msk);

	bytecpy(usrbuf, (uint8_t *)dmabuf_addr, 8ul);
}

/* Copy data to user buffer
 *
 * size_p holds size to copy/copied on input/output
 */
static int numaker_usbd_ep_copy_to_user(struct numaker_usbd_ep *ep_cur, uint8_t *usrbuf,
					     uint32_t *size_p)
{
	const struct device *dev = ep_cur->dev;
	uint32_t dmabuf_addr;

	__ASSERT_NO_MSG(size_p);
	__ASSERT_NO_MSG(ep_cur->dmabuf_valid);

	dmabuf_addr = numaker_usbd_buf_base(dev) + ep_cur->read_fifo_pos;

	/* Clamp to read FIFO used count */
	*size_p = MIN(*size_p, udc_numaker_ep_fifo_used(ep_cur));

	bytecpy(usrbuf, (uint8_t *)dmabuf_addr, *size_p);

	/* Advance read FIFO */
	ep_cur->read_fifo_pos += *size_p;
	ep_cur->read_fifo_used -= *size_p;
	if (ep_cur->read_fifo_used == 0) {
		ep_cur->read_fifo_pos = ep_cur->dmabuf_base;
	}

	return 0;
}

/* Copy data from user buffer
 *
 * size_p holds size to copy/copied on input/output
 */
static int numaker_usbd_ep_copy_from_user(struct numaker_usbd_ep *ep_cur,
					       const uint8_t *usrbuf, uint32_t *size_p)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);
	uint32_t dmabuf_addr;
	uint32_t fifo_free;

	__ASSERT_NO_MSG(size_p);
	__ASSERT_NO_MSG(ep_cur->dmabuf_valid);
	__ASSERT_NO_MSG(ep_cur->mps_valid);
	__ASSERT_NO_MSG(ep_cur->mps <= ep_cur->dmabuf_size);

	dmabuf_addr = numaker_usbd_buf_base(dev) + ep_base->BUFSEG;
	fifo_free = udc_numaker_ep_fifo_max(ep_cur) - udc_numaker_ep_fifo_used(ep_cur);

	*size_p = MIN(*size_p, fifo_free);

	bytecpy((uint8_t *)dmabuf_addr, (uint8_t *)usrbuf, *size_p);

	/* Advance write FIFO */
	ep_cur->write_fifo_pos += *size_p;
	ep_cur->write_fifo_free -= *size_p;
	if (ep_cur->write_fifo_free == 0) {
		ep_cur->write_fifo_pos = ep_cur->dmabuf_base;
	}

	return 0;
}

/* Update EP read/write FIFO on DATA OUT/IN completed */
static void numaker_usbd_ep_fifo_update(struct numaker_usbd_ep *ep_cur)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	__ASSERT_NO_MSG(ep_cur->addr_valid);
	__ASSERT_NO_MSG(ep_cur->dmabuf_valid);

	if (USB_EP_DIR_IS_OUT(ep_cur->addr)) {
		/* Read FIFO */
		/* NOTE: For one-shot implementation, FIFO gets updated from empty. */
		ep_cur->read_fifo_pos = ep_cur->dmabuf_base;
		/* NOTE: See comment on mxpld_ctrlout for why make one copy of CTRL OUT's MXPLD */
		if (USB_EP_GET_IDX(ep_cur->addr) == 0) {
			ep_cur->read_fifo_used = ep_cur->mxpld_ctrlout;
		} else {
			ep_cur->read_fifo_used = ep_base->MXPLD;
		}
	} else {
		/* Write FIFO */
		/* NOTE: For one-shot implementation, FIFO gets to empty. */
		ep_cur->write_fifo_pos = ep_cur->dmabuf_base;
		ep_cur->write_fifo_free = udc_numaker_ep_fifo_max(ep_cur);
	}
}

static void numaker_usbd_ep_config_dmabuf(struct numaker_usbd_ep *ep_cur, uint32_t dmabuf_base,
					  uint32_t dmabuf_size)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	ep_base->BUFSEG = dmabuf_base;

	ep_cur->dmabuf_valid = true;
	ep_cur->dmabuf_base = dmabuf_base;
	ep_cur->dmabuf_size = dmabuf_size;
}

static void numaker_usbd_ep_abort(struct numaker_usbd_ep *ep_cur)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	/* Abort EP on-going transaction */
	ep_base->CFGP |= USBD_CFGP_CLRRDY_Msk;

	/* Need to clear NAK for next transaction */
	ep_cur->nak_clr = false;
}

/* Configure EP major common parts */
static void numaker_usbd_ep_config_major(struct numaker_usbd_ep *ep_cur,
					 struct udc_ep_config *const cfg)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	ep_cur->mps_valid = true;
	ep_cur->mps = cfg->mps;

	/* Configure EP transfer type, DATA0/1 toggle, direction, number, etc. */
	ep_cur->ep_hw_cfg = 0;

	/* Clear STALL Response in Setup stage */
	if ((cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_CONTROL) {
		ep_cur->ep_hw_cfg |= USBD_CFG_CSTALL;
	}

	/* Default to DATA0 */
	ep_cur->ep_hw_cfg &= ~USBD_CFG_DSQSYNC_Msk;

	/* Endpoint IN/OUT, though, default to disabled */
	ep_cur->ep_hw_cfg |= USBD_CFG_EPMODE_DISABLE;

	/* Isochronous or not */
	if ((cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_ISOCHRONOUS) {
		ep_cur->ep_hw_cfg |= USBD_CFG_TYPE_ISO;
	}

	/* Endpoint index */
	ep_cur->ep_hw_cfg |=
		(USB_EP_GET_IDX(cfg->addr) << USBD_CFG_EPNUM_Pos) & USBD_CFG_EPNUM_Msk;

	ep_base->CFG = ep_cur->ep_hw_cfg;
}

static void udc_numaker_ep_enable(struct numaker_usbd_ep *ep_cur)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	/* For safe, EP (re-)enable from clean state */
	numaker_usbd_ep_abort(ep_cur);
	numaker_usbd_ep_clear_stall_n_data_toggle(ep_cur);
	udc_numaker_ep_fifo_reset(ep_cur);

	/* Enable EP to IN/OUT */
	ep_cur->ep_hw_cfg &= ~USBD_CFG_STATE_Msk;
	if (USB_EP_DIR_IS_IN(ep_cur->addr)) {
		ep_cur->ep_hw_cfg |= USBD_CFG_EPMODE_IN;
	} else {
		ep_cur->ep_hw_cfg |= USBD_CFG_EPMODE_OUT;
	}

	ep_base->CFG = ep_cur->ep_hw_cfg;

	/* For USBD, no separate EP interrupt control */
}

static void numaker_usbd_ep_disable(struct numaker_usbd_ep *ep_cur)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	/* For USBD, no separate EP interrupt control */

	/* Disable EP */
	ep_cur->ep_hw_cfg = (ep_cur->ep_hw_cfg & ~USBD_CFG_STATE_Msk) | USBD_CFG_EPMODE_DISABLE;
	ep_base->CFG = ep_cur->ep_hw_cfg;
}

/* Start EP data transaction */
static void udc_numaker_ep_trigger(struct numaker_usbd_ep *ep_cur, uint32_t len)
{
	const struct device *dev = ep_cur->dev;
	USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_cur->ep_hw_idx);

	ep_base->MXPLD = len;
}

static struct numaker_usbd_ep *numaker_usbd_ep_mgmt_alloc_ep(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	struct numaker_usbd_ep_mgmt *ep_mgmt = &priv->ep_mgmt;
	struct numaker_usbd_ep *ep_cur = NULL;

	if (ep_mgmt->ep_idx < priv->ep_pool_size) {
		ep_cur = priv->ep_pool + ep_mgmt->ep_idx;
		ep_mgmt->ep_idx++;

		__ASSERT_NO_MSG(!ep_cur->valid);

		/* Indicate this EP H/W context is allocated */
		ep_cur->valid = true;
	}

	return ep_cur;
}

/* Allocate DMA buffer
 *
 * Return -ENOMEM  on OOM error, or 0 on success with DMA buffer base/size (rounded up) allocated
 */
static int numaker_usbd_ep_mgmt_alloc_dmabuf(const struct device *dev, uint32_t size,
					     uint32_t *dmabuf_base_p, uint32_t *dmabuf_size_p)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	struct numaker_usbd_ep_mgmt *ep_mgmt = &priv->ep_mgmt;

	__ASSERT_NO_MSG(dmabuf_base_p);
	__ASSERT_NO_MSG(dmabuf_size_p);

	/* Required to be 8-byte aligned */
	size = ROUND_UP(size, 8);

	ep_mgmt->dmabuf_pos += size;
	if (ep_mgmt->dmabuf_pos > config->dmabuf_size) {
		ep_mgmt->dmabuf_pos -= size;
		return -ENOMEM;
	}

	*dmabuf_base_p = ep_mgmt->dmabuf_pos - size;
	*dmabuf_size_p = size;
	return 0;
}

/* Initialize all EP H/W contexts */
static void numaker_usbd_ep_mgmt_init(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	USBD_T *const base = config->base;
	struct numaker_usbd_ep_mgmt *ep_mgmt = &priv->ep_mgmt;

	struct numaker_usbd_ep *ep_cur;
	struct numaker_usbd_ep *ep_end;

	/* Initialize all fields to zero except persistent */
	memset(ep_mgmt, 0x00, sizeof(*ep_mgmt));

	ep_cur = priv->ep_pool;
	ep_end = priv->ep_pool + priv->ep_pool_size;

	/* Initialize all EP H/W contexts */
	for (; ep_cur != ep_end; ep_cur++) {
		/* Pointer to the containing device */
		ep_cur->dev = dev;

		/* BSP USBD driver EP handle */
		ep_cur->ep_hw_idx = EP0 + (ep_cur - priv->ep_pool);
	}

	/* Reserve 1st/2nd EP H/W contexts (BSP USBD driver EP0/EP1) for CTRL OUT/IN */
	ep_mgmt->ep_idx = 2;

	/* Reserve DMA buffer for Setup/CTRL OUT/CTRL IN, starting from 0 */
	ep_mgmt->dmabuf_pos = 0;

	/* Configure DMA buffer for Setup packet */
	base->STBUFSEG = ep_mgmt->dmabuf_pos;
	ep_mgmt->dmabuf_pos += NUMAKER_USBD_DMABUF_SIZE_SETUP;

	/* Reserve 1st EP H/W context (BSP USBD driver EP0) for CTRL OUT */
	ep_cur = priv->ep_pool + 0;
	ep_cur->valid = true;
	ep_cur->addr_valid = true;
	ep_cur->addr = USB_EP_GET_ADDR(0, USB_EP_DIR_OUT);
	numaker_usbd_ep_config_dmabuf(ep_cur, ep_mgmt->dmabuf_pos,
				      NUMAKER_USBD_DMABUF_SIZE_CTRLOUT);
	ep_mgmt->dmabuf_pos += NUMAKER_USBD_DMABUF_SIZE_CTRLOUT;
	ep_cur->mps_valid = true;
	ep_cur->mps = NUMAKER_USBD_DMABUF_SIZE_CTRLOUT;

	/* Reserve 2nd EP H/W context (BSP USBD driver EP1) for CTRL IN */
	ep_cur = priv->ep_pool + 1;
	ep_cur->valid = true;
	ep_cur->addr_valid = true;
	ep_cur->addr = USB_EP_GET_ADDR(0, USB_EP_DIR_IN);
	numaker_usbd_ep_config_dmabuf(ep_cur, ep_mgmt->dmabuf_pos, NUMAKER_USBD_DMABUF_SIZE_CTRLIN);
	ep_mgmt->dmabuf_pos += NUMAKER_USBD_DMABUF_SIZE_CTRLIN;
	ep_cur->mps_valid = true;
	ep_cur->mps = NUMAKER_USBD_DMABUF_SIZE_CTRLIN;
}

/* Find EP H/W context by EP address */
static struct numaker_usbd_ep *numaker_usbd_ep_mgmt_find_ep(const struct device *dev,
							    const uint8_t ep)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	struct numaker_usbd_ep_mgmt *ep_mgmt = &priv->ep_mgmt;
	struct numaker_usbd_ep *ep_cur = priv->ep_pool;
	struct numaker_usbd_ep *ep_end = priv->ep_pool + priv->ep_pool_size;

	for (; ep_cur != ep_end; ep_cur++) {
		if (!ep_cur->valid) {
			continue;
		}

		if (!ep_cur->addr_valid) {
			continue;
		}

		if (ep == ep_cur->addr) {
			return ep_cur;
		}
	}

	return NULL;
}

/* Bind EP H/W context to EP address */
static struct numaker_usbd_ep *numaker_usbd_ep_mgmt_bind_ep(const struct device *dev,
							    const uint8_t ep)
{
	struct numaker_usbd_ep *ep_cur = numaker_usbd_ep_mgmt_find_ep(dev, ep);

	if (!ep_cur) {
		ep_cur = numaker_usbd_ep_mgmt_alloc_ep(dev);

		if (!ep_cur) {
			return NULL;
		}

		/* Bind EP H/W context to EP address */
		ep_cur->addr = ep;
		ep_cur->addr_valid = true;
	}

	/* Assert EP H/W context bound to EP address */
	__ASSERT_NO_MSG(ep_cur->valid);
	__ASSERT_NO_MSG(ep_cur->addr_valid);
	__ASSERT_NO_MSG(ep_cur->addr == ep);

	return ep_cur;
}

/* Interrupt bottom half processing for bus reset */
static void numaker_usbd_bus_reset_bh(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	struct numaker_usbd_ep_mgmt *ep_mgmt = &priv->ep_mgmt;

	struct numaker_usbd_ep *ep_cur = priv->ep_pool;
	struct numaker_usbd_ep *ep_end = priv->ep_pool + priv->ep_pool_size;

	for (; ep_cur != ep_end; ep_cur++) {
		/* Reset EP FIFO */
		udc_numaker_ep_fifo_reset(ep_cur);

		/* Abort EP on-going transaction and signal H/W relinquishes DMA buffer ownership */
		numaker_usbd_ep_abort(ep_cur);

		/* Reset EP to unstalled and data toggle bit to 0 */
		numaker_usbd_ep_clear_stall_n_data_toggle(ep_cur);
	}

	numaker_usbd_reset_addr(dev);
}

/* Interrupt bottom half processing for Setup/EP data transaction */
static void udc_numaker_ep_bh(struct numaker_usbd_ep *ep_cur,
			       enum usb_dc_ep_cb_status_code status_code)
{
	const struct device *dev = ep_cur->dev;
	struct udc_numaker_data *priv = udc_get_private(dev);
	struct numaker_usbd_ep_mgmt *ep_mgmt = &priv->ep_mgmt;

	if (status_code == USB_DC_EP_SETUP) {
		/* Zephyr USB device stack passes Setup packet via CTRL OUT EP. */
		__ASSERT_NO_MSG(ep_cur->addr == USB_EP_GET_ADDR(0, USB_EP_DIR_OUT));

		if (udc_numaker_ep_fifo_used(ep_cur)) {
			LOG_WRN("New Setup will override previous Control OUT data");
		}

		/* We should have reserved 1st/2nd EP H/W contexts for CTRL OUT/IN */
		__ASSERT_NO_MSG(ep_cur->addr == USB_EP_GET_ADDR(0, USB_EP_DIR_OUT));
		__ASSERT_NO_MSG((ep_cur + 1)->addr == USB_EP_GET_ADDR(0, USB_EP_DIR_IN));

		/* Reset CTRL OUT/IN FIFO due to new Setup packet */
		udc_numaker_ep_fifo_reset(ep_cur);
		udc_numaker_ep_fifo_reset(ep_cur + 1);

		/* Relinquish CTRL OUT/IN DMA buffer ownership on behalf of H/W */
		numaker_usbd_ep_abort(ep_cur);
		numaker_usbd_ep_abort(ep_cur + 1);

		/* Mark new Setup packet for read */
		numaker_usbd_setup_copy_to_user(dev, (uint8_t *)&ep_mgmt->setup_packet);
		ep_mgmt->new_setup = true;
	} else if (status_code == USB_DC_EP_DATA_OUT) {
		__ASSERT_NO_MSG(USB_EP_DIR_IS_OUT(ep_cur->addr));

		/* Update EP read FIFO */
		numaker_usbd_ep_fifo_update(ep_cur);

		/* Need to clear NAK for next transaction */
		ep_cur->nak_clr = false;
	} else if (status_code == USB_DC_EP_DATA_IN) {
		__ASSERT_NO_MSG(USB_EP_DIR_IS_IN(ep_cur->addr));

		/* Update EP write FIFO */
		numaker_usbd_ep_fifo_update(ep_cur);

		/* Need to clear NAK for next transaction */
		ep_cur->nak_clr = false;
	}
}

/* Message handler for bus udc_event */
static void numaker_usbd_msg_handle_udc_event(const struct device *dev, struct numaker_usbd_msg *msg)
{
	struct udc_numaker_data *priv = udc_get_private(dev);

	__ASSERT_NO_MSG(msg->type == NUMAKER_USBD_MSG_TYPE_UDC_EVENT);

	/* Interrupt bottom half processing for bus reset */
	if (msg->udc_event.type == UDC_EVT_RESET) {
		udc_lock_internal(dev);
		numaker_usbd_bus_reset_bh(dev);
		udc_unlock_internal(dev);
	}

	udc_submit_event(dev, msg->udc_event.type, 0);
}

/* Message handler for Setup transaction completed */
static void numaker_usbd_msg_handle_setup(const struct device *dev, struct numaker_usbd_msg *msg)
{
	struct udc_numaker_data *priv = udc_get_private(dev);

	__ASSERT_NO_MSG(msg->type == NUMAKER_USBD_MSG_TYPE_SETUP);
}

/* Message handler for DATA OUT transaction completed */
static void numaker_usbd_msg_handle_dout(const struct device *dev, struct numaker_usbd_msg *msg)
{
	struct udc_numaker_data *priv = udc_get_private(dev);
	uint8_t ep;
	struct numaker_usbd_ep *ep_cur;

	__ASSERT_NO_MSG(msg->type == NUMAKER_USBD_MSG_TYPE_DOUT);

	ep = msg->dout.ep;

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, ep);
	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", ep);
		return;
	}

	/* Interrupt bottom half processing for EP */
	udc_numaker_ep_bh(ep_cur, msg->cb_ep.status_code);
}

/* Message handler for DATA IN transaction completed */
static void numaker_usbd_msg_handle_din(const struct device *dev, struct numaker_usbd_msg *msg)
{
	struct udc_numaker_data *priv = udc_get_private(dev);
	uint8_t ep;
	struct numaker_usbd_ep *ep_cur;

	__ASSERT_NO_MSG(msg->type == NUMAKER_USBD_MSG_TYPE_DIN);

	ep = msg->din.ep;

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, ep);
	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", ep);
		return;
	}

	/* Interrupt bottom half processing for EP */
	udc_numaker_ep_bh(ep_cur, msg->cb_ep.status_code);
}

/* Message handler for S/W reconnect */
static void numaker_usbd_msg_handle_sw_reconn(const struct device *dev, struct numaker_usbd_msg *msg)
{
	__ASSERT_NO_MSG(msg->type == NUMAKER_USBD_MSG_TYPE_SW_RECONN);

	/* S/W reconnect for error recovery */
	udc_lock_internal(dev);
	numaker_usbd_sw_reconnect(dev);
	udc_unlock_internal(dev);
}

static void numaker_usbd_msg_handler(const struct device *dev)
{
	struct udc_numaker_data *priv = udc_get_private(dev);
	struct numaker_usbd_msg msg;

	while (true) {
		if (k_msgq_get(priv->msgq, &msg, K_FOREVER)) {
			continue;
		}

		udc_lock_internal(dev);

		switch (msg.type) {
		case NUMAKER_USBD_MSG_TYPE_UDC_EVENT:
			numaker_usbd_msg_handle_udc_event(dev, &msg);
			break;

		case NUMAKER_USBD_MSG_TYPE_SETUP:
			numaker_usbd_msg_handle_setup(dev, &msg);
			break;

		case NUMAKER_USBD_MSG_TYPE_DOUT:
			numaker_usbd_msg_handle_dout(dev, &msg);
			break;

		case NUMAKER_USBD_MSG_TYPE_DIN:
			numaker_usbd_msg_handle_din(dev, &msg);
			break;

		case NUMAKER_USBD_MSG_TYPE_XFER:
			/* Just re-activate the message pump */
			break;

		case NUMAKER_USBD_MSG_TYPE_SW_RECONN:
			numaker_usbd_msg_handle_sw_reconn(dev, &msg);
			break;

		default:
			__ASSERT_NO_MSG(false);
		}

		udc_unlock_internal(dev);
	}
}

static void numaker_udbd_isr(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	struct udc_numaker_data *priv = udc_get_private(dev);
	USBD_T *const base = config->base;
	struct numaker_usbd_ep_mgmt *ep_mgmt = &priv->ep_mgmt;

	struct numaker_usbd_msg msg = {0};

	uint32_t volatile usbd_intsts = base->INTSTS;
	uint32_t volatile usbd_bus_state = base->ATTR;

	/* USB plug-in/unplug */
	if (usbd_intsts & USBD_INTSTS_FLDET) {
		/* Floating detect */
		base->INTSTS = USBD_INTSTS_FLDET;

		if (base->VBUSDET & USBD_VBUSDET_VBUSDET_Msk) {
			/* USB plug-in */

			/* Enable back USB/PHY */
			base->ATTR |= USBD_ATTR_USBEN_Msk | USBD_ATTR_PHYEN_Msk;

			/* Message for bottom-half processing */
			msg.type = NUMAKER_USBD_MSG_TYPE_UDC_EVENT;
			msg.udc_event.type = UDC_EVT_VBUS_READY;
			numaker_usbd_send_msg(dev, &msg);

			LOG_DBG("USB plug-in");
		} else {
			/* USB unplug */

			/* Disable USB */
			base->ATTR &= ~USBD_USB_EN;

			/* Message for bottom-half processing */
			msg.type = NUMAKER_USBD_MSG_TYPE_UDC_EVENT;
			msg.udc_event.type = UDC_EVT_VBUS_REMOVED;
			numaker_usbd_send_msg(dev, &msg);

			LOG_DBG("USB unplug");
		}
	}

	/* USB wake-up */
	if (usbd_intsts & USBD_INTSTS_WAKEUP) {
		/* Clear event flag */
		base->INTSTS = USBD_INTSTS_WAKEUP;

		LOG_DBG("USB wake-up");
	}

	/* USB reset/suspend/resume */
	if (usbd_intsts & USBD_INTSTS_BUS) {
		/* Clear event flag */
		base->INTSTS = USBD_INTSTS_BUS;

		if (usbd_bus_state & USBD_STATE_USBRST) {
			/* Bus reset */

			/* Enable back USB/PHY */
			base->ATTR |= USBD_ATTR_USBEN_Msk | USBD_ATTR_PHYEN_Msk;

			/* Bus reset top half */
			numaker_usbd_bus_reset_th(dev);

			/* Message for bottom-half processing */
			msg.type = NUMAKER_USBD_MSG_TYPE_UDC_EVENT;
			msg.udc_event.type = UDC_EVT_RESET;
			numaker_usbd_send_msg(dev, &msg);

			LOG_DBG("USB reset");
		}
		if (usbd_bus_state & USBD_STATE_SUSPEND) {
			/* Enable USB but disable PHY */
			base->ATTR &= ~USBD_PHY_EN;

			/* Message for bottom-half processing */
			msg.type = NUMAKER_USBD_MSG_TYPE_UDC_EVENT;
			msg.udc_event.type = UDC_EVT_SUSPEND;
			numaker_usbd_send_msg(dev, &msg);

			LOG_DBG("USB suspend");
		}
		if (usbd_bus_state & USBD_STATE_RESUME) {
			/* Enable back USB/PHY */
			base->ATTR |= USBD_ATTR_USBEN_Msk | USBD_ATTR_PHYEN_Msk;

			/* Message for bottom-half processing */
			msg.type = NUMAKER_USBD_MSG_TYPE_UDC_EVENT;
			msg.udc_event.type = UDC_EVT_RESUME;
			numaker_usbd_send_msg(dev, &msg);

			LOG_DBG("USB resume");
		}
	}

	/* USB SOF */
	if (usbd_intsts & USBD_INTSTS_SOFIF_Msk) {
		/* Clear event flag */
		base->INTSTS = USBD_INTSTS_SOFIF_Msk;

		/* Message for bottom-half processing */
		msg.type = NUMAKER_USBD_MSG_TYPE_UDC_EVENT;
			msg.udc_event.type = UDC_EVT_SOF;
		numaker_usbd_send_msg(dev, &msg);
	}

	/* USB Setup/EP */
	if (usbd_intsts & USBD_INTSTS_USB) {
		uint32_t epintsts;

		/* Setup event */
		if (usbd_intsts & USBD_INTSTS_SETUP) {
			USBD_EP_T *ep0_base = numaker_usbd_ep_base(dev, EP0);
			USBD_EP_T *ep1_base = numaker_usbd_ep_base(dev, EP1);

			/* Clear event flag */
			base->INTSTS = USBD_INTSTS_SETUP;

			/* Clear the data IN/OUT ready flag of control endpoints */
			ep0_base->CFGP |= USBD_CFGP_CLRRDY_Msk;
			ep1_base->CFGP |= USBD_CFGP_CLRRDY_Msk;

			/* By USB spec, following transactions, regardless of Data/Status stage,
			 * will always be DATA1
			 */
			ep0_base->CFG |= USBD_CFG_DSQSYNC_Msk;
			ep1_base->CFG |= USBD_CFG_DSQSYNC_Msk;

			/* Message for bottom-half processing */
			/* NOTE: In Zephyr USB device stack, Setup packet is passed via
			 * CTRL OUT EP
			 */
			msg.type = NUMAKER_USBD_MSG_TYPE_SETUP;
			numaker_usbd_setup_copy_to_user(dev, msg.setup.packet);
			numaker_usbd_send_msg(dev, &msg);
		}

		/* EP events */
		epintsts = base->EPINTSTS;

		base->EPINTSTS = epintsts;

		while (epintsts) {
			uint32_t ep_hw_idx = u32_count_trailing_zeros(epintsts);
			USBD_EP_T *ep_base = numaker_usbd_ep_base(dev, ep_hw_idx);
			uint8_t ep_dir;
			uint8_t ep_idx;
			uint8_t ep;

			/* We don't enable INNAKEN interrupt, so as long as EP event occurs,
			 * we can just regard one data transaction has completed (ACK for
			 * CTRL/BULK/INT or no-ACK for Iso), that is, no need to check EPSTS0,
			 * EPSTS1, etc.
			 */

			/* EP direction, number, and address */
			ep_dir = ((ep_base->CFG & USBD_CFG_STATE_Msk) == USBD_CFG_EPMODE_IN)
					 ? USB_EP_DIR_IN
					 : USB_EP_DIR_OUT;
			ep_idx = (ep_base->CFG & USBD_CFG_EPNUM_Msk) >> USBD_CFG_EPNUM_Pos;
			ep = USB_EP_GET_ADDR(ep_idx, ep_dir);

			/* NOTE: See comment in usb_dc_set_address()'s implementation
			 * for safe place to change USB device address
			 */
			if (ep == USB_EP_GET_ADDR(0, USB_EP_DIR_IN)) {
				numaker_usbd_set_addr(dev);
			}

			/* NOTE: See comment on mxpld_ctrlout for why make one copy of
			 * CTRL OUT's MXPLD
			 */
			if (ep == USB_EP_GET_ADDR(0, USB_EP_DIR_OUT)) {
				struct numaker_usbd_ep *ep_ctrlout = priv->ep_pool + 0;
				USBD_EP_T *ep_ctrlout_base =
					numaker_usbd_ep_base(dev, ep_ctrlout->ep_hw_idx);

				ep_ctrlout->mxpld_ctrlout = ep_ctrlout_base->MXPLD;
			}

			/* Message for bottom-half processing */
			if (USB_EP_DIR_IS_OUT(ep)) {
				msg.type = NUMAKER_USBD_MSG_TYPE_DOUT;
				msg.dout.ep = ep;
			} else {
				msg.type = NUMAKER_USBD_MSG_TYPE_DIN;
				msg.din.ep = ep;
			}
			numaker_usbd_send_msg(dev, &msg);

			/* Have handled this EP and go next */
			epintsts &= ~BIT(ep_hw_idx);
		}
	}
}

int usb_dc_ep_flush(const uint8_t ep)
{
	const struct device *dev = numaker_usbd_device_get();
	int err = 0;
	struct numaker_usbd_ep *ep_cur;

	LOG_DBG("ep=0x%02x", ep);

	udc_lock_internal(dev);

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, ep);

	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", ep);
		err = -ENOMEM;
		goto cleanup;
	}

	udc_numaker_ep_fifo_reset(ep_cur);

cleanup:

	udc_unlock_internal(dev);

	return err;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data_buf, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
	const struct device *dev = numaker_usbd_device_get();
	int err = 0;
	struct numaker_usbd_ep *ep_cur;
	uint32_t data_len_act;

	udc_lock_internal(dev);

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, ep);

	if (!ep_cur) {
		LOG_ERR("ep=0x%02x", ep);
		err = -ENOMEM;
		goto cleanup;
	}

	if (!USB_EP_DIR_IS_IN(ep)) {
		LOG_ERR("Invalid EP address 0x%02x for write", ep);
		err = -EINVAL;
		goto cleanup;
	}

	/* For USBD, avoid duplicate NAK clear */
	if (ep_cur->nak_clr) {
		LOG_WRN("ep 0x%02x busy", ep);
		err = -EAGAIN;
		goto cleanup;
	}

	/* For one-shot implementation, don't trigger next DATA IN with write FIFO not empty. */
	if (udc_numaker_ep_fifo_used(ep_cur)) {
		LOG_WRN("ep 0x%02x: Write FIFO not empty for one-shot implementation", ep);
		err = -EAGAIN;
		goto cleanup;
	}

	/* NOTE: Null data or zero data length are valid, used for ZLP */
	if (data_buf && data_len) {
		data_len_act = data_len;
		err = numaker_usbd_ep_copy_from_user(ep_cur, data_buf, &data_len_act);
		if (err < 0) {
			LOG_ERR("Copy to FIFO from user buffer");
			goto cleanup;
		}
	} else {
		data_len_act = 0;
	}

	/* Now H/W actually owns EP DMA buffer */
	udc_numaker_ep_trigger(ep_cur, data_len_act);

	/* NOTE: For one-shot implementation, at most MPS size can be written, though,
	 * null 'ret_bytes' requires all data written.
	 */
	if (ret_bytes) {
		*ret_bytes = data_len_act;
	} else if (data_len_act != data_len) {
		LOG_ERR("Expected write all %d bytes, but actual %d bytes written", data_len,
			data_len_act);
		err = -EIO;
		goto cleanup;
	}

cleanup:

	udc_unlock_internal(dev);

	return err;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	const struct device *dev = numaker_usbd_device_get();
	int err = 0;

	udc_lock_internal(dev);

	err = usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes);
	if (err < 0) {
		goto cleanup;
	}

	err = usb_dc_ep_read_continue(ep);
	if (err < 0) {
		goto cleanup;
	}

cleanup:

	udc_unlock_internal(dev);

	return err;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data_buf, uint32_t max_data_len, uint32_t *read_bytes)
{
	const struct device *dev = numaker_usbd_device_get();
	struct udc_numaker_data *priv = udc_get_private(dev);
	int err = 0;
	struct numaker_usbd_ep_mgmt *ep_mgmt = &priv->ep_mgmt;
	struct numaker_usbd_ep *ep_cur;
	uint32_t data_len_act = 0;

	udc_lock_internal(dev);

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, ep);

	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", ep);
		err = -ENOMEM;
		goto cleanup;
	}

	if (!USB_EP_DIR_IS_OUT(ep)) {
		LOG_ERR("Invalid EP address 0x%02x for read", ep);
		err = -EINVAL;
		goto cleanup;
	}

	/* Special handling for USB_CONTROL_EP_OUT on Setup packet */
	if (ep == USB_CONTROL_EP_OUT && ep_mgmt->new_setup) {
		if (!data_buf || max_data_len != 8) {
			LOG_ERR("Invalid parameter for reading Setup packet");
			err = -EINVAL;
			goto cleanup;
		}

		memcpy(data_buf, &ep_mgmt->setup_packet, 8);
		ep_mgmt->new_setup = false;

		if (read_bytes) {
			*read_bytes = 8;
		}

		goto cleanup;
	}

	/* For one-shot implementation, don't read FIFO with EP busy. */
	if (ep_cur->nak_clr) {
		LOG_WRN("ep 0x%02x busy", ep);
		err = -EAGAIN;
		goto cleanup;
	}

	/* NOTE: Null data and zero data length is valid, used for returning number of
	 * available bytes for read
	 */
	if (data_buf) {
		data_len_act = max_data_len;
		err = numaker_usbd_ep_copy_to_user(ep_cur, data_buf, &data_len_act);
		if (err < 0) {
			LOG_ERR("Copy from FIFO to user buffer");
			goto cleanup;
		}

		if (read_bytes) {
			*read_bytes = data_len_act;
		}
	} else if (max_data_len) {
		LOG_ERR("Null data but non-zero data length");
		err = -EINVAL;
		goto cleanup;
	} else {
		if (read_bytes) {
			*read_bytes = udc_numaker_ep_fifo_used(ep_cur);
		}
	}

	/* Suppress further USB_DC_EP_DATA_OUT events by replying NAK or disabling interrupt
	 *
	 * For USBD, further control is unnecessary because NAK is automatically replied until
	 * next USBD_SET_PAYLOAD_LEN().
	 */

cleanup:

	udc_unlock_internal(dev);

	return err;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	const struct device *dev = numaker_usbd_device_get();
	int err = 0;
	struct numaker_usbd_ep *ep_cur;

	udc_lock_internal(dev);

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, ep);

	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", ep);
		err = -ENOMEM;
		goto cleanup;
	}

	if (!USB_EP_DIR_IS_OUT(ep)) {
		LOG_ERR("Invalid EP address 0x%02x for read", ep);
		err = -EINVAL;
		goto cleanup;
	}

	/* Avoid duplicate NAK clear */
	if (ep_cur->nak_clr) {
		err = 0;
		goto cleanup;
	}

	/* For one-shot implementation, don't trigger next DATA OUT, or overwrite. */
	if (udc_numaker_ep_fifo_used(ep_cur)) {
		goto cleanup;
	}

	__ASSERT_NO_MSG(ep_cur->mps_valid);
	udc_numaker_ep_trigger(ep_cur, ep_cur->mps);

cleanup:

	udc_unlock_internal(dev);

	return err;
}

static enum udc_bus_speed udc_numaker_device_speed(const struct device *dev)
{
	return UDC_BUS_SPEED_FS;
}

static int udc_numaker_ep_enqueue(const struct device *dev,
				   struct udc_ep_config *const cfg,
				   struct net_buf *buf)
{
	struct numaker_usbd_msg msg = {0};

	LOG_DBG("%p enqueue %p", dev, buf);
	udc_buf_put(cfg, buf);

	/* Resume the EP's queued transfer */
	if (!cfg->stat.halted) {
		msg.type = NUMAKER_USBD_MSG_TYPE_XFER;
		msg.xfer.ep = cfg->addr;
		numaker_usbd_send_msg(dev, &msg);
	}

	return 0;
}

static int udc_numaker_ep_dequeue(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;
	struct numaker_usbd_ep *ep_cur;

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, cfg->addr);
	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", cfg->addr);
		return -ENOMEM;
	}

	numaker_usbd_ep_abort(ep_cur);

	lock_key = irq_lock();

	buf = udc_buf_get_all(dev, cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	irq_unlock(lock_key);

	return 0;
}

static int udc_numaker_ep_set_halt(const struct device *dev,
				    struct udc_ep_config *const cfg)
{
	struct numaker_usbd_ep *ep_cur;
	USBD_EP_T *ep_base;

	LOG_DBG("Set halt ep 0x%02x", cfg->addr);

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, cfg->addr);
	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", cfg->addr);
		return -ENOMEM;
	}

	/* Set EP to stalled */
	numaker_usbd_ep_set_stall(ep_cur);
	cfg->stat.halted = true;

	return 0;
}

static int udc_numaker_ep_clear_halt(const struct device *dev,
				      struct udc_ep_config *const cfg)
{
	struct numaker_usbd_ep *ep_cur;
	struct numaker_usbd_msg msg = {0};

	LOG_DBG("Clear halt ep 0x%02x", cfg->addr);

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, cfg->addr);
	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", cfg->addr);
		return -ENOMEM;
	}

	/* Reset EP to unstalled and data toggle bit to 0 */
	numaker_usbd_ep_clear_stall_n_data_toggle(ep_cur);
	cfg->stat.halted = false;

	/* Resume the EP's queued transfer */
	msg.type = NUMAKER_USBD_MSG_TYPE_XFER;
	msg.xfer.ep = cfg->addr;
	numaker_usbd_send_msg(dev, &msg);

	return 0;
}
		     
static int udc_numaker_ep_enable(const struct device *dev,
				  struct udc_ep_config *const cfg)
{
	int err;
	uint32_t dmabuf_base;
	uint32_t dmabuf_size;
	struct numaker_usbd_ep *ep_cur;

	LOG_DBG("Enable ep 0x%02x", cfg->addr);

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, cfg->addr);
	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", cfg->addr);
		return -ENOMEM;
	}

	/* Configure EP DMA buffer */
	if (!ep_cur->dmabuf_valid || ep_cur->dmabuf_size < cfg->mps) {
		/* Allocate DMA buffer */
		err = numaker_usbd_ep_mgmt_alloc_dmabuf(dev, cfg->mps, &dmabuf_base,
						       &dmabuf_size);
		if (err < 0) {
			LOG_ERR("Allocate DMA buffer failed");
			return err;
		}

		/* Configure EP DMA buffer */
		numaker_usbd_ep_config_dmabuf(ep_cur, dmabuf_base, dmabuf_size);
	}

	/* Configure EP majorly */
	numaker_usbd_ep_config_major(ep_cur, cfg);

	/* Enable EP */
	numaker_usbd_ep_enable(ep_cur);

	/* FIXME */
	/* Trigger OUT transaction manually, or H/W will continue to reply NAK because
	 * Zephyr USB device stack is unclear on kicking off by invoking usb_dc_ep_read()
	 * or friends. We needn't do this for CTRL OUT because Setup sequence will involve
	 * this.
	 */
	if (USB_EP_DIR_IS_OUT(cfg->addr) && USB_EP_GET_IDX(cfg->addr) != 0) {
		err = usb_dc_ep_read_continue(cfg->addr);
		if (err < 0) {
			return err;
		}
	}

	return 0;
}

static int udc_numaker_ep_disable(const struct device *dev,
				   struct udc_ep_config *const cfg)
{
	int err;
	struct numaker_usbd_ep *ep_cur;

	LOG_DBG("Disable ep 0x%02x", cfg->addr);

	/* Bind EP H/W context to EP address */
	ep_cur = numaker_usbd_ep_mgmt_bind_ep(dev, cfg->addr);
	if (!ep_cur) {
		LOG_ERR("Bind EP H/W context: ep=0x%02x", cfg->addr);
		return -ENOMEM;
	}

	/* Disable EP */
	numaker_usbd_ep_disable(ep_cur);

	return 0;
}

static int udc_numaker_host_wakeup(const struct device *dev)
{
	const struct udc_numaker_config *config = dev->config;
	USBD_T *const base = config->base;

	/* Enable back USB/PHY first */
	base->ATTR |= USBD_ATTR_USBEN_Msk | USBD_ATTR_PHYEN_Msk;

	/* Then generate 'K' */
	base->ATTR |= USBD_ATTR_RWAKEUP_Msk;
	k_sleep(K_USEC(NUMAKER_USBD_BUS_RESUME_DRV_K_US));
	base->ATTR ^= USBD_ATTR_RWAKEUP_Msk;

	return 0;
}

static int udc_numaker_set_address(const struct device *dev, const uint8_t addr)
{
	struct udc_numaker_data *priv = udc_get_private(dev);

	LOG_DBG("Set new address %u for %p", addr, dev);

	/* NOTE: Timing for configuring USB device address into H/W is critical. It must be done
	 * in-between SET_ADDRESS control transfer and next transfer. For this, it is done in
	 * IN ACK ISR of SET_ADDRESS control transfer.
	 */
	priv->addr = addr;

	return 0;
}

static int udc_numaker_enable(const struct device *dev)
{
	LOG_DBG("Enable device %p", dev);

	/* S/W connect */
	numaker_usbd_sw_connect(dev);

	return 0;
}

static int udc_numaker_disable(const struct device *dev)
{
	LOG_DBG("Enable device %p", dev);

	/* S/W disconnect */
	numaker_usbd_sw_disconnect(dev);

	return 0;
}

static int udc_numaker_init(const struct device *dev)
{
	int err;

	/* Initialize USBD H/W */
	err = numaker_usbd_hw_setup(dev);
	if (err < 0) {
		LOG_ERR("Set up H/W: %d", err);
		return err;
	}

	/* USB device address defaults to 0 */
	numaker_usbd_reset_addr(dev);

	/* Initialize all EP contexts */
	numaker_usbd_ep_mgmt_init(dev);

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN,
				   USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	return 0;
}

static int udc_numaker_shutdown(const struct device *dev)
{
	struct udc_numaker_data *priv = udc_get_private(dev);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	/* Uninitialize USBD H/W */
	numaker_usbd_hw_shutdown(numaker_usbd_device_get());

	/* Purge message queue */
	k_msgq_purge(priv->msgq);

	return 0;
}

static int udc_numaker_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_numaker_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

static int udc_numaker_driver_preinit(const struct device *dev)
{
	const struct udc_skeleton_config *config = dev->config;
	struct udc_data *data = dev->data;
	struct udc_numaker_data *priv = udc_get_private(dev);
	int err;

	data->caps.rwup = true;
	data->caps.addr_before_status = true;
	data->caps.mps0 = UDC_MPS0_64;

	/* Some soc series don't allow ISO IN/OUT to be assigned the same EP number.
	 * This is addressed by limiting all OUT/IN EP addresses in top/bottom halves,
	 * except CTRL OUT/IN.
	 */

	for (int i = 0; i < config->ep_cfg_out_size; i++) {
		/* Limit all OUT EP numbers to 0, 1~7 */
		if (config->disallow_iso_inout_same && i != 0 && i >= 8) {
			continue;
		}

		config->ep_cfg_out[i].caps.out = 1;
		if (i == 0) {
			config->ep_cfg_out[i].caps.control = 1;
			config->ep_cfg_out[i].caps.mps = 64;
		} else {
			config->ep_cfg_out[i].caps.bulk = 1;
			config->ep_cfg_out[i].caps.interrupt = 1;
			config->ep_cfg_out[i].caps.iso = 1;
			config->ep_cfg_out[i].caps.mps = 1023;
		}

		config->ep_cfg_out[i].addr = USB_EP_DIR_OUT | i;
		err = udc_register_ep(dev, &config->ep_cfg_out[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	for (int i = 0; i < config->ep_cfg_in_size; i++) {
		/* Limit all IN EP numbers to 0, 8~15 */
		if (config->disallow_iso_inout_same && i != 0 && i < 8) {
			continue;
		}

		config->ep_cfg_in[i].caps.in = 1;
		if (i == 0) {
			config->ep_cfg_in[i].caps.control = 1;
			config->ep_cfg_in[i].caps.mps = 64;
		} else {
			config->ep_cfg_in[i].caps.bulk = 1;
			config->ep_cfg_in[i].caps.interrupt = 1;
			config->ep_cfg_in[i].caps.iso = 1;
			config->ep_cfg_in[i].caps.mps = 1023;
		}

		config->ep_cfg_in[i].addr = USB_EP_DIR_IN | i;
		err = udc_register_ep(dev, &config->ep_cfg_in[i]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
	}

	config->make_thread(dev);

	return 0;
}

static const struct udc_api udc_numaker_api = {
	.device_speed = udc_numaker_device_speed,
	.ep_enqueue = udc_numaker_ep_enqueue,
	.ep_dequeue = udc_numaker_ep_dequeue,
	.ep_set_halt = udc_numaker_ep_set_halt,
	.ep_clear_halt = udc_numaker_ep_clear_halt,
	.ep_enable = udc_numaker_ep_enable,
	.ep_disable = udc_numaker_ep_disable,
	.host_wakeup = udc_numaker_host_wakeup,
	.set_address = udc_numaker_set_address,
	.enable = udc_numaker_enable,
	.disable = udc_numaker_disable,
	.init = udc_numaker_init,
	.shutdown = udc_numaker_shutdown,
	.lock = udc_numaker_lock,
	.unlock = udc_numaker_unlock,
};

#define UDC_NUMAKER_DEVICE_DEFINE(inst)                                                                  \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	static void udc_numaker_irq_config_func_##inst(const struct device *dev)                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), numaker_udbd_isr,     \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
                                                                                                   \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}                                                                                          \
                                                                                                   \
	static void udc_numaker_irq_unconfig_func_##inst(const struct device *dev)               \
	{                                                                                          \
		irq_disable(DT_INST_IRQN(inst));                                                   \
	}                                                                                          \
                                                                                                   \
	K_THREAD_STACK_DEFINE(udc_numaker_stack_##inst, UDC_NUMAKER_THREAD_STACK_SIZE);	\
										\
	static void udc_numaker_thread_##inst(void *dev, void *arg1, void *arg2)	\
	{									\
		ARG_UNUSED(arg1);                                               \
		ARG_UNUSED(arg2);                                               \
		numaker_usbd_msg_handler(dev);					\
	}									\
										\
	static void udc_numaker_make_thread_##inst(const struct device *dev)	\
	{									\
		struct udc_numaker_data *priv = udc_get_private(dev);		\
										\
		k_thread_create(&priv->thread_data,				\
				udc_numaker_stack_##inst,				\
				K_THREAD_STACK_SIZEOF(udc_numaker_stack_##n),	\
				udc_numaker_thread_##inst,			\
				(void *)dev, NULL, NULL,			\
				K_PRIO_COOP(CONFIG_UDC_NUMAKER_THREAD_PRIORITY),\
				K_ESSENTIAL,					\
				K_NO_WAIT);					\
		k_thread_name_set(&priv->thread_data, dev->name);		\
	}									\
	\
	static struct udc_ep_config						\
		ep_cfg_out_##inst[MIN(DT_INST_PROP(inst, num_bidir_endpoints), 16)]; \
	static struct udc_ep_config					\
		ep_cfg_in_##inst[MIN(DT_INST_PROP(inst, num_bidir_endpoints), 16)];		\
										\
	static const struct udc_numaker_config udc_numaker_config_##inst = {                     \
		.ep_cfg_out = ep_cfg_out_##inst, \
		.ep_cfg_in = ep_cfg_in_##inst,	\
		.ep_cfg_out_size = ARRAY_SIZE(ep_cfg_out_##inst), \
		.ep_cfg_in_size = ARRAY_SIZE(ep_cfg_in_##inst), \
		.make_thread = udc_numaker_make_thread_##inst,			\
		.base = (USBD_T *)DT_INST_REG_ADDR(inst),                                          \
		.reset = RESET_DT_SPEC_INST_GET(inst),                                             \
		.clk_modidx = DT_INST_CLOCKS_CELL(inst, clock_module_index),                       \
		.clk_src = DT_INST_CLOCKS_CELL(inst, clock_source),                                \
		.clk_div = DT_INST_CLOCKS_CELL(inst, clock_divider),                               \
		.clkctrl_dev = DEVICE_DT_GET(DT_PARENT(DT_INST_CLOCKS_CTLR(inst))),                \
		.irq_config_func = udc_numaker_irq_config_func_##inst,                            \
		.irq_unconfig_func = udc_numaker_irq_unconfig_func_##inst,                       \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                    \
		.dmabuf_size = DT_INST_PROP(inst, dma_buffer_size),                                \
		.disallow_iso_inout_same = DT_INST_PROP(inst, disallow_iso_in_out_same_number),    \
	};                                                                                         \
                                                                                                   \
	static struct numaker_usbd_ep numaker_usbd_ep_pool_##inst[DT_INST_PROP(inst, num_bidir_endpoints)];		\
	\
	K_MSGQ_DEFINE(numaker_usbd_msgq_##inst, sizeof(struct numaker_usbd_msg), UDC_NUMAKER_MSG_QUEUE_SIZE, 4); \
	\
	static struct udc_numaker_data udc_priv_##inst = {                                  \
		.msgq = &numaker_usbd_msgq_##inst, \
		.ep_pool = numaker_usbd_ep_pool_##inst, \
		.ep_pool_size = DT_INST_PROP(inst, num_bidir_endpoints), \
	};			\
	\
	static struct udc_data udc_data_##inst = {					\
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##inst.mutex),		\
		.priv = &udc_priv_##inst,						\
	};									\
	\
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, udc_numaker_driver_preinit, NULL, &udc_data_##inst,            \
			      &udc_numaker_config_##inst, POST_KERNEL,                            \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, udc_numaker_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_NUMAKER_DEVICE_DEFINE)
