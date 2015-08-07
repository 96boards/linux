/*
 * Hisilicon mailbox definition
 *
 * Copyright (c) 2015 Hisilicon Limited.
 * Copyright (c) 2015 Linaro Limited.
 *
 * Author: Leo Yan <leo.yan@linaro.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __HISI_MBOX_H
#define __HISI_MBOX_H

#include <linux/device.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/mailbox_controller.h>
#include <linux/spinlock.h>

#define MBOX_MAX_CHANS		32
#define MBOX_RX			0x0
#define MBOX_TX			0x1

struct hisi_mbox_queue {
	spinlock_t lock;
	struct kfifo fifo;
	struct work_struct work;
	struct mbox_chan *chan;
	bool full;
};

struct hisi_mbox_chan {

	/*
	 * Description for channel's hardware info:
	 * - direction;
	 * - peer core id for communication;
	 * - local irq vector or number;
	 * - remoted irq vector or number for peer core;
	 */
	unsigned int dir;
	unsigned int peer_core;
	unsigned int remote_irq;
	unsigned int local_irq;

	/*
	 * Slot address is cached value derived from index
	 * within buffer for every channel
	 */
	void __iomem *slot;

	/* For rx's fifo operations */
	struct hisi_mbox_queue *mq;

	struct hisi_mbox_hw *mbox_hw;
};

struct hisi_mbox_ops {
	int (*startup)(struct hisi_mbox_chan *chan);
	void (*shutdown)(struct hisi_mbox_chan *chan);

	int (*rx)(struct hisi_mbox_chan *chan, void *msg, int len);
	int (*tx)(struct hisi_mbox_chan *chan, void *msg, int len);
	int (*tx_is_done)(struct hisi_mbox_chan *chan);
	int (*ack)(struct hisi_mbox_chan *chan);

	u32 (*get_pending_irq)(struct hisi_mbox_hw *mbox_hw);

	void (*clear_irq)(struct hisi_mbox_chan *chan);
	void (*enable_irq)(struct hisi_mbox_chan *chan);
	void (*disable_irq)(struct hisi_mbox_chan *chan);
};

struct hisi_mbox_hw {
	struct device *dev;

	unsigned int irq;

	/* flag of enabling tx's irq mode */
	bool tx_irq_mode;

	/* region for ipc event */
	void __iomem *ipc;

	/* region for share mem */
	void __iomem *buf;

	unsigned int chan_num;
	struct hisi_mbox_chan *chan;
	struct hisi_mbox_ops *ops;
	struct hisi_mbox *parent;
};

struct hisi_mbox {
	struct hisi_mbox_hw *hw;

	void *irq_map_chan[MBOX_MAX_CHANS];
	struct mbox_chan *chan;
	struct mbox_controller controller;
};

extern int hisi_mbox_register(struct hisi_mbox_hw *mbox_hw);
extern void hisi_mbox_unregister(struct hisi_mbox_hw *mbox_hw);

#endif /* __HISI_MBOX_H */
