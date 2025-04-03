/*
 * Copyright (c) 2025 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_CLOCK_CONTROL_TISCI_CLOCK_CONTROL_H_
#define ZEPHYR_DRIVERS_CLOCK_CONTROL_TISCI_CLOCK_CONTROL_H_

/**
 * @struct clock_config
 * @brief Clock configuration structure
 *
 * This structure is used to define the configuration for a clock, including
 * the device ID and clock ID.
 *
 * @var clock_config::dev_id
 * Device ID associated with the clock.
 *
 * @var clock_config::clk_id
 * Clock ID within the device.
 */
struct clock_config {
	uint32_t dev_id;
	uint32_t clk_id;
};

#endif