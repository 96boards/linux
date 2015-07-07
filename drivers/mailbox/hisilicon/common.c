/*
 * Hisilicon mailbox common driver
 *
 * This is skeleton driver for Hisilicon's mailbox, it registers mailbox
 * channels into framework; it also need invoke low level's callback
 * functions for low level operations. RX channel's message queue is
 * based on the code written in drivers/mailbox/omap-mailbox.c.

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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include "common.h"

#define MBOX_MSG_LEN		(32)
#define MBOX_MSG_NUM		(16)
#define MBOX_MSG_FIFO_SIZE	(MBOX_MSG_LEN * MBOX_MSG_NUM)

static bool hisi_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct hisi_mbox_chan *mchan = chan->con_priv;
	struct hisi_mbox_hw *mbox_hw = mchan->mbox_hw;

	/* Only set idle state for polling mode */
	BUG_ON(mbox_hw->tx_irq_mode);

	return mbox_hw->ops->tx_is_done(mchan);
}

static int hisi_mbox_send_data(struct mbox_chan *chan, void *msg)
{
	struct hisi_mbox_chan *mchan = chan->con_priv;
	struct hisi_mbox_hw *mbox_hw = mchan->mbox_hw;

	return mbox_hw->ops->tx(mchan, msg, MBOX_MSG_LEN);
}

static void hisi_mbox_rx_work(struct work_struct *work)
{
	struct hisi_mbox_queue *mq =
			container_of(work, struct hisi_mbox_queue, work);
	struct mbox_chan *chan = mq->chan;
	struct hisi_mbox_chan *mchan = chan->con_priv;
	struct hisi_mbox_hw *mbox_hw = mchan->mbox_hw;

	char msg[MBOX_MSG_LEN];
	int len;

	while (kfifo_len(&mq->fifo) >= sizeof(msg)) {
		len = kfifo_out(&mq->fifo, (unsigned char *)&msg, sizeof(msg));
		WARN_ON(len != sizeof(msg));

		mbox_chan_received_data(chan, (void *)msg);
		spin_lock_irq(&mq->lock);
		if (mq->full) {
			mq->full = false;
			mbox_hw->ops->enable_irq(mchan);
		}
		spin_unlock_irq(&mq->lock);
	}
}

static void hisi_mbox_tx_interrupt(struct mbox_chan *chan)
{
	struct hisi_mbox_chan *mchan = chan->con_priv;
	struct hisi_mbox_hw *mbox_hw = mchan->mbox_hw;

	mbox_hw->ops->clear_irq(mchan);
	mbox_hw->ops->ack(mchan);

	mbox_chan_txdone(chan, 0);
}

static void hisi_mbox_rx_interrupt(struct mbox_chan *chan)
{
	struct hisi_mbox_chan *mchan = chan->con_priv;
	struct hisi_mbox_queue *mq = mchan->mq;
	struct hisi_mbox_hw *mbox_hw = mchan->mbox_hw;
	char msg[MBOX_MSG_LEN];
	int len;

	if (unlikely(kfifo_avail(&mq->fifo) < sizeof(msg))) {
		mbox_hw->ops->disable_irq(mchan);
		mq->full = true;
		goto nomem;
	}

	mbox_hw->ops->rx(mchan, msg, sizeof(msg));

	len = kfifo_in(&mq->fifo, (unsigned char *)&msg, sizeof(msg));
	WARN_ON(len != sizeof(msg));

	mbox_hw->ops->ack(mchan);
nomem:
	schedule_work(&mq->work);
}

static irqreturn_t hisi_mbox_interrupt(int irq, void *p)
{
	struct hisi_mbox *mbox = p;
	struct hisi_mbox_hw *mbox_hw = mbox->hw;
	struct hisi_mbox_chan *mchan;
	struct mbox_chan *chan;
	unsigned int state;
	unsigned int intr_bit;

	state = mbox_hw->ops->get_pending_irq(mbox_hw);
	if (!state) {
		dev_warn(mbox_hw->dev, "%s: spurious interrupt\n",
			 __func__);
		return IRQ_HANDLED;
	}

	while (state) {
		intr_bit = __ffs(state);
		state &= (state - 1);

		chan = mbox->irq_map_chan[intr_bit];
		if (!chan) {
			dev_warn(mbox_hw->dev, "%s: unexpected irq vector %d\n",
				 __func__, intr_bit);
			continue;
		}

		mchan = chan->con_priv;
		if (mchan->dir == MBOX_TX)
			hisi_mbox_tx_interrupt(chan);
		else
			hisi_mbox_rx_interrupt(chan);
	}

	return IRQ_HANDLED;
}

static struct hisi_mbox_queue *hisi_mbox_queue_alloc(
		struct mbox_chan *chan,
		void (*work)(struct work_struct *))
{
	struct hisi_mbox_queue *mq;

	if (!work)
		return NULL;

	mq = kzalloc(sizeof(struct hisi_mbox_queue), GFP_KERNEL);
	if (!mq)
		return NULL;

	spin_lock_init(&mq->lock);

	if (kfifo_alloc(&mq->fifo, MBOX_MSG_FIFO_SIZE, GFP_KERNEL))
		goto error;

	mq->chan = chan;
	INIT_WORK(&mq->work, work);
	return mq;

error:
	kfree(mq);
	return NULL;
}

static void hisi_mbox_queue_free(struct hisi_mbox_queue *mq)
{
	kfifo_free(&mq->fifo);
	kfree(mq);
}

static int hisi_mbox_startup(struct mbox_chan *chan)
{
	struct hisi_mbox_chan *mchan = chan->con_priv;
	struct hisi_mbox_hw *mbox_hw = mchan->mbox_hw;
	struct hisi_mbox *mbox = mchan->mbox_hw->parent;

	struct hisi_mbox_queue *mq;
	unsigned int irq = mchan->local_irq;

	mq = hisi_mbox_queue_alloc(chan, hisi_mbox_rx_work);
	if (!mq)
		return -ENOMEM;
	mchan->mq = mq;

	mbox->irq_map_chan[irq] = (void *)chan;
	return mbox_hw->ops->startup(mchan);
}

static void hisi_mbox_shutdown(struct mbox_chan *chan)
{
	struct hisi_mbox_chan *mchan = chan->con_priv;
	struct hisi_mbox_hw *mbox_hw = mchan->mbox_hw;
	struct hisi_mbox *mbox = mbox_hw->parent;
	unsigned int irq = mchan->local_irq;

	mbox_hw->ops->shutdown(mchan);

	mbox->irq_map_chan[irq] = NULL;
	flush_work(&mchan->mq->work);
	hisi_mbox_queue_free(mchan->mq);
}

static struct mbox_chan_ops hisi_mbox_chan_ops = {
	.send_data    = hisi_mbox_send_data,
	.startup      = hisi_mbox_startup,
	.shutdown     = hisi_mbox_shutdown,
	.last_tx_done = hisi_mbox_last_tx_done,
};

int hisi_mbox_register(struct hisi_mbox_hw *mbox_hw)
{
	struct device *dev = mbox_hw->dev;
	struct hisi_mbox *mbox;
	int i, err;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->hw = mbox_hw;
	mbox->chan = devm_kzalloc(dev,
		mbox_hw->chan_num * sizeof(struct mbox_chan), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	err = devm_request_irq(dev, mbox_hw->irq, hisi_mbox_interrupt, 0,
			dev_name(dev), mbox);
	if (err) {
		dev_err(dev, "Failed to register a mailbox IRQ handler: %d\n",
			err);
		return -ENODEV;
	}

	for (i = 0; i < mbox_hw->chan_num; i++) {
		mbox->chan[i].con_priv = &mbox_hw->chan[i];
		mbox->irq_map_chan[i] = NULL;
	}

	mbox->controller.dev = dev;
	mbox->controller.chans = &mbox->chan[0];
	mbox->controller.num_chans = mbox_hw->chan_num;
	mbox->controller.ops = &hisi_mbox_chan_ops;

	if (mbox_hw->tx_irq_mode)
		mbox->controller.txdone_irq = true;
	else {
		mbox->controller.txdone_poll = true;
		mbox->controller.txpoll_period = 5;
	}

	err = mbox_controller_register(&mbox->controller);
	if (err) {
		dev_err(dev, "Failed to register mailbox %d\n", err);
		return err;
	}

	mbox_hw->parent = mbox;
	return 0;
}
EXPORT_SYMBOL_GPL(hisi_mbox_register);

void hisi_mbox_unregister(struct hisi_mbox_hw *mbox_hw)
{
	struct hisi_mbox *mbox = mbox_hw->parent;

	mbox_controller_unregister(&mbox->controller);
}
EXPORT_SYMBOL_GPL(hisi_mbox_unregister);
