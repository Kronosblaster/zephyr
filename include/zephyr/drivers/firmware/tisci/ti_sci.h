/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */
 #ifndef INCLUDE_ZEPHYR_DRIVERS_TISCI_H_
 #define INCLUDE_ZEPHYR_DRIVERS_TISCI_H_
 
 #include <zephyr/device.h>
 
 int ti_sci_cmd_clk_get_freq(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				 uint64_t *freq);
 int ti_sci_cmd_clk_set_freq(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				 uint64_t min_freq, uint64_t target_freq, uint64_t max_freq);
 int ti_sci_cmd_get_clock(const struct device *dev, uint32_t dev_id, uint8_t clk_id, bool needs_ssc,
			  bool can_change_freq, bool enable_input_term);
 int ti_sci_cmd_idle_clock(const struct device *dev, uint32_t dev_id, uint8_t clk_id);
 int ti_sci_cmd_get_revision(const struct device *dev);
 #endif