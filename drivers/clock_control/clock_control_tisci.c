/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <stdint.h>

#include <zephyr/drivers/firmware/ti_sci/ti_sci.h>
#include <zephyr/drivers/clock_control.h>

#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>

/**
 * ti_sci_cmd_get_clock_state() - Get clock state helper
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @programmed_state:	State requested for clock to move to
 * @current_state:	State that the clock is currently in
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_get_clock_state(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				      uint8_t *programmed_state, uint8_t *current_state)
{
	struct ti_sci_msg_resp_get_clock_state *resp;
	struct ti_sci_msg_req_get_clock_state req;
	struct ti_sci_xfer *xfer;
	int ret = 0;
	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_GET_CLOCK_STATE,
				     TI_SCI_FLAG_REQ_ACK_ON_PROCESSED, (uint32_t *)&req,
				     sizeof(req), sizeof(*resp));
	req.dev_id = dev_id;
	req.clk_id = clk_id;
	ret = ti_sci_do_xfer(dev, xfer);
	resp = (struct ti_sci_msg_resp_get_clock_state *)rx_message.buf;
	if (programmed_state) {
		*programmed_state = resp->programmed_state;
	}
	if (current_state) {
		*current_state = resp->current_state;
	}
	return ret;
}

/**
 * ti_sci_set_clock_state() - Set clock state helper
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @flags:	Flags to be used for this request
 * @state:	State requested for clock to move to
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_set_clock_state(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				  uint32_t flags, uint8_t state)
{
	struct ti_sci_msg_resp_get_clock_state *resp;
	struct ti_sci_msg_req_set_clock_state req;
	struct ti_sci_xfer *xfer;
	int ret = 0;

	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_SET_CLOCK_STATE,
				     flags | TI_SCI_FLAG_REQ_ACK_ON_PROCESSED, (uint32_t *)&req,
				     sizeof(req), sizeof(*resp));
	req.dev_id = dev_id;
	req.clk_id = clk_id;
	req.request_state = state;

	ret = ti_sci_do_xfer(dev, xfer);

	return ret;
}

/**
 * ti_sci_cmd_get_clock() - Get control of a clock from TI SCI
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @needs_ssc: 'true' if Spread Spectrum clock is desired, else 'false'
 * @can_change_freq: 'true' if frequency change is desired, else 'false'
 * @enable_input_term: 'true' if input termination is desired, else 'false'
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_get_clock(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				bool needs_ssc, bool can_change_freq, bool enable_input_term)
{
	uint32_t flags = 0;

	flags |= needs_ssc ? MSG_FLAG_CLOCK_ALLOW_SSC : 0;
	flags |= can_change_freq ? MSG_FLAG_CLOCK_ALLOW_FREQ_CHANGE : 0;
	flags |= enable_input_term ? MSG_FLAG_CLOCK_INPUT_TERM : 0;

	return ti_sci_set_clock_state(dev, dev_id, clk_id, flags, MSG_CLOCK_SW_STATE_REQ);
}

/**
 * ti_sci_cmd_idle_clock() - Idle a clock which is in our control
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 *
 * NOTE: This clock must have been requested by get_clock previously.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_idle_clock(const struct device *dev, uint32_t dev_id, uint8_t clk_id)
{
	return ti_sci_set_clock_state(dev, dev_id, clk_id, 0, MSG_CLOCK_SW_STATE_UNREQ);
}

/**
 * ti_sci_cmd_put_clock() - Release a clock from our control back to TISCI
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 *
 * NOTE: This clock must have been requested by get_clock previously.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_put_clock(const struct device *dev, uint32_t dev_id, uint8_t clk_id)
{
	return ti_sci_set_clock_state(dev, dev_id, clk_id, 0, MSG_CLOCK_SW_STATE_AUTO);
}

/**
 * ti_sci_cmd_clk_is_auto() - Is the clock being auto managed
 * @handle:	pointer to TI SCI handle
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @req_state: state indicating if the clock is auto managed
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_is_auto(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				  bool *req_state)
{
	uint8_t state = 0;
	int ret;

	if (!req_state) {
		return -EINVAL;
	}

	ret = ti_sci_cmd_get_clock_state(dev, dev_id, clk_id, &state, NULL);
	if (ret) {
		return ret;
	}

	*req_state = (state == MSG_CLOCK_SW_STATE_AUTO);
	return 0;
}

/**
 * ti_sci_cmd_clk_is_on() - Is the clock ON
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @req_state: state indicating if the clock is managed by us and enabled
 * @curr_state: state indicating if the clock is ready for operation
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_is_on(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				bool *req_state, bool *curr_state)
{
	uint8_t c_state = 0, r_state = 0;
	int ret;

	if (!req_state && !curr_state) {
		return -EINVAL;
	}

	ret = ti_sci_cmd_get_clock_state(dev, dev_id, clk_id, &r_state, &c_state);
	if (ret) {
		return ret;
	}

	if (req_state) {
		*req_state = (r_state == MSG_CLOCK_SW_STATE_REQ);
	}
	if (curr_state) {
		*curr_state = (c_state == MSG_CLOCK_HW_STATE_READY);
	}
	return 0;
}

/**
 * ti_sci_cmd_clk_is_off() - Is the clock OFF
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @req_state: state indicating if the clock is managed by us and disabled
 * @curr_state: state indicating if the clock is NOT ready for operation
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_is_off(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				 bool *req_state, bool *curr_state)
{
	uint8_t c_state = 0, r_state = 0;
	int ret;

	if (!req_state && !curr_state) {
		return -EINVAL;
	}

	ret = ti_sci_cmd_get_clock_state(dev, dev_id, clk_id, &r_state, &c_state);
	if (ret) {
		return ret;
	}

	if (req_state) {
		*req_state = (r_state == MSG_CLOCK_SW_STATE_UNREQ);
	}
	if (curr_state) {
		*curr_state = (c_state == MSG_CLOCK_HW_STATE_NOT_READY);
	}
	return 0;
}

/**
 * ti_sci_cmd_clk_set_parent() - Set the clock source of a specific device clock
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @parent_id:	Parent clock identifier to set
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_set_parent(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				     uint8_t parent_id)
{
	struct ti_sci_msg_req_set_clock_parent req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	int ret = 0;

	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_SET_CLOCK_PARENT,
				     TI_SCI_FLAG_REQ_ACK_ON_PROCESSED, (uint32_t *)&req,
				     sizeof(req), sizeof(*resp));
	req.dev_id = dev_id;
	req.clk_id = clk_id;
	req.parent_id = parent_id;

	ret = ti_sci_do_xfer(dev, xfer);
	if (ret) {
		return ret;
	}

	return ret;
}

/**
 * ti_sci_cmd_clk_get_parent() - Get current parent clock source
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @parent_id:	Current clock parent
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_get_parent(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				     uint8_t *parent_id)
{
	struct ti_sci_msg_resp_get_clock_parent *resp;
	struct ti_sci_msg_req_get_clock_parent req;
	struct ti_sci_xfer *xfer;
	int ret = 0;
	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_GET_CLOCK_PARENT,
				     TI_SCI_FLAG_REQ_ACK_ON_PROCESSED, (uint32_t *)&req,
				     sizeof(req), sizeof(*resp));

	req.dev_id = dev_id;
	req.clk_id = clk_id;

	ret = ti_sci_do_xfer(dev, xfer);

	*parent_id = resp->parent_id;

	return ret;
}

/**
 * ti_sci_cmd_clk_get_num_parents() - Get num parents of the current clk source
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @num_parents: Returns he number of parents to the current clock.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_get_num_parents(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
					  uint8_t *num_parents)
{
	struct ti_sci_msg_resp_get_clock_num_parents *resp;
	struct ti_sci_msg_req_get_clock_num_parents req;
	struct ti_sci_xfer *xfer;
	int ret = 0;

	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_GET_NUM_CLOCK_PARENTS,
				     TI_SCI_FLAG_REQ_ACK_ON_PROCESSED, (uint32_t *)&req,
				     sizeof(req), sizeof(*resp));
	req.dev_id = dev_id;
	req.clk_id = clk_id;

	ret = ti_sci_do_xfer(dev, xfer);
	if (ret) {
		return ret;
	}

	resp = (struct ti_sci_msg_resp_get_clock_num_parents *)rx_message.buf;

	*num_parents = resp->num_parents;

	return ret;
}

/**
 * ti_sci_cmd_clk_get_match_freq() - Find a good match for frequency
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @min_freq:	The minimum allowable frequency in Hz. This is the minimum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @target_freq: The target clock frequency in Hz. A frequency will be
 *		processed as close to this target frequency as possible.
 * @max_freq:	The maximum allowable frequency in Hz. This is the maximum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @match_freq:	Frequency match in Hz response.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_get_match_freq(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
					 uint64_t min_freq, uint64_t target_freq, uint64_t max_freq,
					 uint64_t *match_freq)
{
	struct ti_sci_msg_resp_query_clock_freq *resp;
	struct ti_sci_msg_req_query_clock_freq req;
	struct ti_sci_xfer *xfer;
	int ret = 0;
	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_QUERY_CLOCK_FREQ,
				     TI_SCI_FLAG_REQ_ACK_ON_PROCESSED, (uint32_t *)&req,
				     sizeof(req), sizeof(*resp));
	req.dev_id = dev_id;
	req.clk_id = clk_id;
	req.min_freq_hz = min_freq;
	req.target_freq_hz = target_freq;
	req.max_freq_hz = max_freq;

	ret = ti_sci_do_xfer(dev, xfer);
	if (ret) {
		return ret;
	}

	resp = (struct ti_sci_msg_resp_query_clock_freq *)rx_message.buf;

	*match_freq = resp->freq_hz;

	return ret;
}

/**
 * ti_sci_cmd_clk_set_freq() - Set a frequency for clock
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @min_freq:	The minimum allowable frequency in Hz. This is the minimum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 * @target_freq: The target clock frequency in Hz. A frequency will be
 *		processed as close to this target frequency as possible.
 * @max_freq:	The maximum allowable frequency in Hz. This is the maximum
 *		allowable programmed frequency and does not account for clock
 *		tolerances and jitter.
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_set_freq(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				   uint64_t min_freq, uint64_t target_freq, uint64_t max_freq)
{
	struct ti_sci_msg_req_set_clock_freq req;
	struct ti_sci_msg_hdr *resp;
	struct ti_sci_xfer *xfer;
	int ret = 0;

	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_SET_CLOCK_FREQ,
				     TI_SCI_FLAG_REQ_ACK_ON_PROCESSED, (uint32_t *)&req,
				     sizeof(req), sizeof(*resp));
	req.dev_id = dev_id;
	req.clk_id = clk_id;
	req.min_freq_hz = min_freq;
	req.target_freq_hz = target_freq;
	req.max_freq_hz = max_freq;

	ret = ti_sci_do_xfer(dev, xfer);
	if (ret) {
		return ret;
	}

	return ret;
}

/**
 * ti_sci_cmd_clk_get_freq() - Get current frequency
 * @dev:	pointer to TI SCI dev
 * @dev_id:	Device identifier this request is for
 * @clk_id:	Clock identifier for the device for this request.
 *		Each device has it's own set of clock inputs. This indexes
 *		which clock input to modify.
 * @freq:	Currently frequency in Hz
 *
 * Return: 0 if all went well, else returns appropriate error value.
 */
static int ti_sci_cmd_clk_get_freq(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
				   uint64_t *freq)
{
	struct ti_sci_msg_resp_get_clock_freq *resp;
	struct ti_sci_msg_req_get_clock_freq req;
	struct ti_sci_xfer *xfer;
	int ret = 0;

	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_GET_CLOCK_FREQ,
				     TI_SCI_FLAG_REQ_ACK_ON_PROCESSED, (uint32_t *)&req,
				     sizeof(req), sizeof(*resp));
	req.dev_id = dev_id;
	req.clk_id = clk_id;

	ret = ti_sci_do_xfer(dev, xfer);
	if (ret) {
		return ret;
	}

	resp = (struct ti_sci_msg_resp_get_clock_freq *)rx_message.buf;

	*freq = resp->freq_hz;

	return ret;
}

// Define the structure if not already defined elsewhere
struct ti_sci_clock_control_subsys {
	uint32_t dev_id;
	uint8_t clk_id;
};

static int tisci_clock_control_get_rate(const struct device *dev, clock_control_subsys_t sys,
					uint32_t *rate)
{
	ti_sci_cmd_clk_get_freq(dev, ((struct ti_sci_clock_control_subsys *)sys)->dev_id,
				((struct ti_sci_clock_control_subsys *)sys)->clk_id,
				(uint64_t *)rate);
}
static DEVICE_API(clock_control, tisci_clock_driver_api) = {
	.get_rate = tisci_clock_control_get_rate,
};

static int tisci_clock_init(const struct device *dev)
{

	printk("clock found");
	return 0;
}
#define TISCI_INIT(inst)                                                                           \
	DEVICE_DT_INST_DEFINE(inst, tisci_clock_init, NULL, NULL, NULL, PRE_KERNEL_1,              \
			      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &tisci_clock_driver_api);
DT_INST_FOREACH_STATUS_OKAY(TISCI_INIT)