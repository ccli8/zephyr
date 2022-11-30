/*
 * Copyright (c) 2023 Nuvoton Technology Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_WIFI)

#include <stdio.h>
#include <stdlib.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>

#include "wifi.h"

K_SEM_DEFINE(wifi_connect_sem, 0, 1);
static struct net_mgmt_event_callback wifi_connect_cb;
bool wifi_connected;

static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;

	wifi_connected = (status->status == 0);
	k_sem_give(&wifi_connect_sem);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
				    struct net_if *iface)
{
	switch (mgmt_event) {
	case NET_EVENT_WIFI_CONNECT_RESULT:
		handle_wifi_connect_result(cb);
		break;

	default:
		break;
	}
}

bool wifi_connect(void)
{
	/* Prepare for waiting on WiFi connect */
	k_sem_reset(&wifi_connect_sem);

	/* Register event callback for WiFi connect */
	net_mgmt_init_event_callback(&wifi_connect_cb, wifi_mgmt_event_handler,
				     NET_EVENT_WIFI_CONNECT_RESULT);
	net_mgmt_add_event_callback(&wifi_connect_cb);

	/* Wait on WiFi connected */
	k_sem_take(&wifi_connect_sem, K_FOREVER);

	net_mgmt_del_event_callback(&wifi_connect_cb);

	k_msleep(500);

	return wifi_connected;
}

#endif
