/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/sys/printk.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/misc/ti_sci/ti_sci.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

struct rx_msg {
	uint8_t seq;
	uint8_t size;
	char buf[256];
};
struct rx_msg rx_message;

struct ti_sci_xfer {
	struct mbox_msg tx_message;
	struct rx_msg rx_message;
	uint8_t rx_len;
};

struct ti_sci_version_info {
	uint8_t abi_major;
	uint8_t abi_minor;
	uint16_t firmware_revision;
	char firmware_description[32];
};

struct ti_sci_desc {
	uint8_t default_host_id;
	int max_rx_timeout_ms;
	int max_msgs;
	int max_msg_size;
};

struct tisci_data {
	struct ti_sci_xfer xfer;
	struct ti_sci_desc desc;
	struct ti_sci_version_info version;
	uint32_t host_id;
	uint8_t seq;
};

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

	struct ti_sci_msg_resp_version *rev_info;
	rev_info = (struct ti_sci_msg_resp_version *)rx_message.buf;
	printk("DMSC Firmware Description: %s\n", rev_info->firmware_description);
	printk("DMSC Firmware Version: 0x%x\n",rev_info->firmware_revision);
	printk("DMSC ABI Version: %d.%d\n", rev_info->abi_major, rev_info->abi_minor);
	
	
}

void print_message_byte_by_byte(const void *message, size_t size)
{
	const uint8_t *byte_ptr = (const uint8_t *)message;
	for (size_t i = 0; i < size; ++i) {
		printk("Byte %zu: 0x%02X\n", i, byte_ptr[i]);
	}
}

void print_binary(uint32_t value)
{
	for (int i = 31; i >= 0; --i) {
		printk("%c", (value & (1 << i)) ? '1' : '0');
		if (i % 8 == 0) {
			printk(" ");
		}
	}
	printk("\n");
}

struct __packed tisci_header {
	struct ti_sci_msg_hdr requestHeader;
	struct ti_sci_secure_msg_hdr secureHeader;
};

struct __packed tisci_message {
	struct tisci_header header;
	uint8_t padding[44];
};

void print_message_fields(const struct ti_sci_msg_hdr *header)
{
	for (uint8_t i = 0; i < 8; i++) {
		unsigned char byte = ((unsigned char *)header)[i];
		printk("%x\n", byte);
	}
	printk("\n");
}

int main(void)
{
	const struct mbox_dt_spec tx_channel = MBOX_DT_SPEC_GET(DT_NODELABEL(dmsc), tx);
	const struct mbox_dt_spec rx_channel = MBOX_DT_SPEC_GET(DT_NODELABEL(dmsc), rx);

	struct ti_sci_msg_hdr requestHeader = {.seq = 2,
					       .type = TI_SCI_MSG_VERSION,
					       .host = 36,
					       .flags = TI_SCI_FLAG_REQ_ACK_ON_PROCESSED};

	struct ti_sci_secure_msg_hdr secureHeader = {.checksum = 0, .reserved = 0};
	struct tisci_header header_instance = {.requestHeader = requestHeader,
					       .secureHeader = secureHeader};
	struct tisci_message message_instance = {.header = header_instance, .padding = {0}};

	printk("mbox_data Client demo started\n");

	if (mbox_register_callback_dt(&rx_channel, callback, NULL)) {
		printk("mbox_register_callback() error\n");
		return 0;
	}

	if (mbox_set_enabled_dt(&rx_channel, true)) { // Enable the mailbox
		printk("mbox_set_enable() error\n");
		return 0;
	}

	printk("Sending version request (on channel %d)\n", tx_channel.channel_id);

	struct mbox_msg msg;
	msg.data = (void *)(&message_instance);
	msg.size = sizeof(message_instance);

	if (mbox_send_dt(&tx_channel, &msg) < 0) {
		printk("mbox_send() error\n");
		return 0;
	}
	printk("mbox_data Client demo ended\n");
	return 0;
}
