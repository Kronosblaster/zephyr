/*
 * Copyright (c) 2025, Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <stdint.h>
#include <stdio.h>
#define DT_DRV_COMPAT ti_k2g_sci

#include <zephyr/drivers/mbox.h>
#include <zephyr/device.h>
#include <zephyr/drivers/firmware/tisci/tisci_protocol.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ti, k2g - sci);


/* Received message container */
static struct rx_msg rx_message;

/* Semaphore for signaling response readiness */
static struct k_sem response_ready_sem;

/**
 * struct tisci_config - TISCI device configuration
 * @mbox_tx: Mailbox transmit channel
 * @mbox_rx: Mailbox receive channel
 * @host_id: Host ID for the device
 */
struct tisci_config {
	struct mbox_dt_spec mbox_tx;
	struct mbox_dt_spec mbox_rx;
	uint32_t host_id;
};

/**
 * struct rx_msg - Received message details
 * @seq: Message sequence number
 * @size: Message size in bytes
 * @buf: Buffer for message data
 */
struct rx_msg {
	uint8_t seq;
	uint8_t size;
	char buf[256];
};

/**
 * struct ti_sci_xfer - TISCI transfer details
 * @tx_message: Transmit message
 * @rx_message: Received message
 * @rx_len: Expected receive message length
 */
struct ti_sci_xfer {
	struct mbox_msg tx_message;
	struct rx_msg rx_message;
	uint8_t rx_len;
};

/**
 * struct tisci_data - Runtime data for TISCI device
 * @xfer: Current transfer details
 * @desc: Device descriptor
 * @version: Firmware version info
 * @host_id: Host ID for the device
 * @seq: Current transfer sequence number
 */
struct tisci_data {
	struct ti_sci_xfer xfer;
	struct ti_sci_desc desc;
	struct ti_sci_version_info version;
	uint32_t host_id;
	uint8_t seq;
};

/**
 * ti_sci_setup_one_xfer() - Prepare a TISCI transfer
 * @dev: Pointer to the TISCI device
 * @msg_type: Message type for the transfer
 * @msg_flags: Flags for the transfer (e.g., acknowledgment requirements)
 * @buf: Pointer to the buffer containing the message payload
 * @tx_message_size: Size of the transmit message in bytes
 * @rx_message_size: Size of the expected receive message in bytes
 *
 * This function initializes a TISCI transfer by setting up the transmit and
 * receive message buffers, assigning a sequence number, and populating the
 * message header with the appropriate metadata (e.g., message type, host ID,
 * and flags). It ensures that the transfer sizes are within valid limits.
 *
 * Return: Pointer to the initialized transfer structure on success, or NULL
 *         if the transfer sizes are invalid.
 */
struct ti_sci_xfer *ti_sci_setup_one_xfer(const struct device *dev, uint16_t msg_type,
					  uint32_t msg_flags, void *buf, size_t tx_message_size,
					  size_t rx_message_size)
{
	struct tisci_data *data = dev->data;
	struct ti_sci_xfer *xfer = &data->xfer;
	struct ti_sci_msg_hdr *hdr;

	/* Ensure we have sane transfer sizes */
	if (rx_message_size > data->desc.max_msg_size ||
	    tx_message_size > data->desc.max_msg_size ||
	    (rx_message_size > 0 && rx_message_size < sizeof(*hdr)) ||
	    tx_message_size < sizeof(*hdr)) {
		return NULL;
	}

	data->seq++;
	xfer->tx_message.data = buf;
	xfer->tx_message.size = tx_message_size;
	xfer->rx_len = (uint8_t)rx_message_size;

	hdr = (struct ti_sci_msg_hdr *)buf;
	hdr->seq = data->seq;
	hdr->type = msg_type;
	hdr->host = data->host_id;
	hdr->flags = msg_flags;

	return xfer;
}

/**
 * callback() - Mailbox callback for handling TISCI firmware responses
 * @dev: Pointer to the mailbox device
 * @channel_id: Mailbox channel ID
 * @user_data: User-defined data (unused in this implementation)
 * @data: Pointer to the received mailbox message
 *
 * This function is triggered when a response is received via the mailbox.
 * It processes the incoming message, copies it to the global `rx_message`
 * structure, and signals the semaphore to notify the waiting thread.
 */
static void callback(const struct device *dev, mbox_channel_id_t channel_id, void *user_data,
	      struct mbox_msg *data)
{
	const struct ti_sci_msg_hdr *hdr = data->data;

	if (data->size > 64) {
		LOG_ERR("Too large incoming message");
	}

	memcpy(rx_message.buf, data->data, 256);
	rx_message.size = data->size;
	rx_message.seq = hdr->seq;

	k_sem_give(&response_ready_sem);
}

/**
 * ti_sci_is_response_ack() - Generic ACK/NACK message checkup
 * @r:	pointer to response buffer
 *
 * Return: true if the response was an ACK, else returns false.
 */
 static bool ti_sci_is_response_ack(void *r)
 {
	 struct ti_sci_msg_hdr *hdr = (struct ti_sci_msg_hdr *)r;
 
	 return hdr->flags & TI_SCI_FLAG_RESP_GENERIC_ACK ? true : false;
 }

 /**
 * ti_sci_get_response() - Wait for and validate TISCI firmware response
 * @dev: Pointer to the TISCI device
 * @xfer: Pointer to the transfer structure
 *
 * Waits for a response using a semaphore and validates it by checking the
 * sequence number, message size, and other parameters.
 *
 * Return: 0 on success, or an error code:
 *         - -ETIMEDOUT: Timeout waiting for response
 *         - -EINVAL: Invalid response (e.g., mismatched sequence or size)
 */
static int ti_sci_get_response(const struct device *dev, struct ti_sci_xfer *xfer)
{
	struct tisci_data *dev_data = dev->data;
	struct ti_sci_msg_hdr *hdr;
	int ret = 0;
	
	// hack
	if (k_sem_take(&response_ready_sem, K_MSEC(100)) != 0) {
        LOG_ERR("Timeout waiting for response");
        return -ETIMEDOUT;
    }

	hdr = (struct ti_sci_msg_hdr *)rx_message.buf;

	/* Sanity check for message response */
	if (hdr->seq != dev_data->seq) {
		LOG_ERR("HDR seq != data seq [%d != %d]\n", hdr->seq, dev_data->seq);
		return -EINVAL;
	}

	if (rx_message.size > dev_data->desc.max_msg_size) {
		LOG_ERR("rx_message.size [ %d ] > max_msg_size\n", xfer->rx_message.size);
		return -EINVAL;
	}

	if (rx_message.size < xfer->rx_len) {
		LOG_ERR("rx_message.size [ %d ] < xfer->rx_len\n", xfer->rx_message.size);
		return -EINVAL;
	}

	return ret;
}

/**
 * ti_sci_do_xfer() - Do one transfer
 * @info:	Pointer to SCI entity information
 * @xfer:	Transfer to initiate and wait for response
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
static int ti_sci_do_xfer(const struct device *dev, struct ti_sci_xfer *xfer)
{
	const struct tisci_config *config = dev->config;
	struct mbox_msg *msg = &xfer->tx_message;
	int ret;

	ret = mbox_send_dt(&config->mbox_tx, msg);
	if (ret < 0) {
		LOG_ERR("Could not send (%d)\n", ret);
		return 0;
	}

	/* Get response if requested */
	if (xfer->rx_len) {
		ret = ti_sci_get_response(dev, xfer);
		if (!ti_sci_is_response_ack(rx_message.buf)) {
			LOG_ERR("TISCI Response in NACK\n");
			ret = -ENODEV;
		}
	}

	return ret;
}

int ti_sci_cmd_get_revision(const struct device *dev)
{
	struct tisci_data *data = dev->data;
	struct ti_sci_msg_hdr hdr;
	struct ti_sci_version_info *ver;
	struct ti_sci_msg_resp_version *rev_info;
	struct ti_sci_xfer *xfer;

	xfer = ti_sci_setup_one_xfer(dev, TI_SCI_MSG_VERSION, TI_SCI_FLAG_REQ_ACK_ON_PROCESSED,
				     &hdr, sizeof(struct ti_sci_msg_hdr), sizeof(*rev_info));

	ti_sci_do_xfer(dev, xfer);

	rev_info = (struct ti_sci_msg_resp_version *)rx_message.buf;
	ver = &data->version;
	ver->abi_major = rev_info->abi_major;
	ver->abi_minor = rev_info->abi_minor;
	ver->firmware_revision = rev_info->firmware_revision;
	printf("%s", rev_info->firmware_description);
	strncpy(ver->firmware_description, rev_info->firmware_description,
		sizeof(ver->firmware_description));
	return 0;
}

int ti_sci_cmd_get_clock_state(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
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

int ti_sci_cmd_clk_is_auto(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
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

int ti_sci_cmd_clk_is_on(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
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

int ti_sci_cmd_clk_is_off(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
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
int ti_sci_cmd_clk_get_match_freq(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
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

int ti_sci_cmd_clk_set_freq(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
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

int ti_sci_cmd_clk_get_freq(const struct device *dev, uint32_t dev_id, uint8_t clk_id,
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

static int tisci_init(const struct device *dev)
{	
	const struct tisci_config *config = dev->config;
	struct tisci_data *data = dev->data;
	int ret;

	k_sem_init(&response_ready_sem, 0, 1);

	data->host_id = config->host_id;
	data->seq = 0x0;
	data->desc.default_host_id = config->host_id;
	data->desc.max_rx_timeout_ms = 1000;
	data->desc.max_msgs = 5;
	data->desc.max_msg_size = 60;

	ret = mbox_register_callback_dt(&config->mbox_rx, callback, NULL);
	if (ret < 0) {
		printk("Could not register callback (%d)\n", ret);
		return 0;
	}

	ret = mbox_set_enabled_dt(&config->mbox_rx, true);
	if (ret < 0) {
		printk("Could not enable RX channel (%d)\n", ret);
		return 0;
	}
	return 0;
}

#define TISCI_DEFINE(_n)                                                                           \
	static struct tisci_data tisci_data_##_n;                                                  \
	static const struct tisci_config tisci_config_##_n = {                                     \
		.mbox_tx = MBOX_DT_SPEC_INST_GET(_n, tx),                                          \
		.mbox_rx = MBOX_DT_SPEC_INST_GET(_n, rx),                                          \
		.host_id = DT_INST_PROP(_n, ti_host_id),                                           \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(_n, tisci_init, NULL, &tisci_data_##_n, &tisci_config_##_n,          \
			      PRE_KERNEL_1, CONFIG_TISCI_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TISCI_DEFINE)
