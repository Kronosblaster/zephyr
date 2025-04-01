/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/firmware/ti_sci/tisci_protocol.h>

struct ti_sci_xfer;
struct tisci_data;

extern struct ti_sci_xfer *ti_sci_setup_one_xfer(const struct device *dev, uint16_t msg_type,
						 uint32_t msg_flags, void *buf,
						 size_t tx_message_size, size_t rx_message_size);

extern int ti_sci_get_response(const struct device *dev, struct ti_sci_xfer *xfer);

extern  bool ti_sci_is_response_ack(void *r);

extern int ti_sci_do_xfer(const struct device *dev, struct ti_sci_xfer *xfer);

extern  int ti_sci_cmd_get_revision(const struct device *dev);

extern int tisci_init(const struct device *dev);

extern char* tisci_receive();