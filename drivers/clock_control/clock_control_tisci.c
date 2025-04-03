/*
 * Copyright (c) 2025, Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#define DT_DRV_COMPAT ti_k2g_sci_clk

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/firmware/tisci/ti_sci.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/tisci_clock_control.h>
#include <zephyr/devicetree.h>
#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ti_k2g_sci_clk);

const struct device *dmsc = DEVICE_DT_GET(DT_NODELABEL(dmsc));

static int tisci_get_rate(const struct device *dev, clock_control_subsys_t sys, uint32_t *rate)
{
	struct clock_config *req = (struct clock_config *)sys;
	ti_sci_cmd_clk_get_freq(dmsc, req->dev_id, req->clk_id, (uint64_t *)rate);
	return 0;
}

static int tisci_set_rate(const struct device *dev, void *sys, void *rate)
{
	struct clock_config *req = (struct clock_config *)sys;
	uint64_t freq = *((uint64_t *)rate);
	ti_sci_cmd_clk_set_freq(dmsc, req->dev_id, req->clk_id, freq, freq, freq);
	return 0;
}

static inline int tisci_on(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_config *req = (struct clock_config *)sys;
	return ti_sci_cmd_get_clock(dmsc, req->dev_id, req->clk_id, false, true, false);
}

static inline int tisci_off(const struct device *dev, clock_control_subsys_t sys)
{
	struct clock_config *req = (struct clock_config *)sys;
	return ti_sci_cmd_idle_clock(dmsc, req->dev_id, req->clk_id);
}

static DEVICE_API(clock_control, tisci_clock_driver_api) = {
	.get_rate = tisci_get_rate,
	.set_rate = tisci_set_rate,
	.on = tisci_on,
	.off = tisci_off
};

#define TI_K2G_SCI_CLK_INIT(_n)                                                                    \
	DEVICE_DT_INST_DEFINE(_n, NULL, NULL, NULL, NULL, PRE_KERNEL_1,                            \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &tisci_clock_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TI_K2G_SCI_CLK_INIT)