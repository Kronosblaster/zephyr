/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <stdint.h>
#define DT_DRV_COMPAT ti_k2g_sci

#include <zephyr/device.h>
#include <zephyr/drivers/firmware/ti_sci/tisci_protocol.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ti, k2g - sci);

struct rx_msg rx_message;

char* ti_sci_receive(){
	return rx_message.buf;
}

struct ti_sci_xfer *ti_sci_setup_one_xfer(const struct device *dev, uint16_t msg_type,
						 uint32_t msg_flags, void *buf,
						 size_t tx_message_size, size_t rx_message_size)
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

void callback(const struct device *dev, mbox_channel_id_t channel_id, void *user_data,
		     struct mbox_msg *data)
{
	const struct ti_sci_msg_hdr *hdr = data->data;

	if (data->size > 64) {
		LOG_ERR("Too large incoming message");
	}

	memcpy(rx_message.buf, data->data, 256);
	rx_message.size = data->size;
	rx_message.seq = hdr->seq;
}

int ti_sci_get_response(const struct device *dev, struct ti_sci_xfer *xfer)
{
	struct tisci_data *dev_data = dev->data;
	struct ti_sci_msg_hdr *hdr;
	int ret = 0;

	/* hack */
	k_sleep(K_MSEC(10));

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
 * ti_sci_is_response_ack() - Generic ACK/NACK message checkup
 * @r:	pointer to response buffer
 *
 * Return: true if the response was an ACK, else returns false.
 */
bool ti_sci_is_response_ack(void *r)
{
	struct ti_sci_msg_hdr *hdr = (struct ti_sci_msg_hdr *)r;

	return hdr->flags & TI_SCI_FLAG_RESP_GENERIC_ACK ? true : false;
}

/**
 * ti_sci_do_xfer() - Do one transfer
 * @info:	Pointer to SCI entity information
 * @xfer:	Transfer to initiate and wait for response
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
int ti_sci_do_xfer(const struct device *dev, struct ti_sci_xfer *xfer)
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

/**
 * ti_sci_cmd_get_revision() - command to get the revision of the SCI entity
 * @dev:	pointer to TI SCI dev
 *
 * Updates the SCI information in the internal data structure.
 *
 * Return: 0 if all went fine, else return appropriate error.
 */
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
	strncpy(ver->firmware_description, rev_info->firmware_description,
		sizeof(ver->firmware_description));
	return 0;
}

int tisci_init(const struct device *dev)
{
	const struct tisci_config *config = dev->config;
	struct tisci_data *data = dev->data;
	int ret;

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
			      POST_KERNEL, CONFIG_TISCI_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TISCI_DEFINE)
