/*
 * Hisilicon mailbox driver
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

#include <linux/device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#define MBOX_MAX_CHANS			32

#define MBOX_RX				0x0
#define MBOX_TX				0x1

#define MBOX_STATUS_MASK		(0xF << 4)
#define MBOX_STATUS_IDLE		(0x1 << 4)
#define MBOX_STATUS_TX			(0x2 << 4)
#define MBOX_STATUS_RX			(0x4 << 4)
#define MBOX_STATUS_ACK			(0x8 << 4)

#define MBOX_ACK_CONFIG_MASK		(0x1 << 0)
#define MBOX_ACK_AUTOMATIC		(0x1 << 0)
#define MBOX_ACK_IRQ			(0x0 << 0)

#define MBOX_MODE_OFF			(0x0)
#define MBOX_DATA_OFF(idx)		(0x4 + (idx << 2))

#define MBOX_INT_RAW_OFF(core)		(0x400 + (core << 4))
#define MBOX_INT_MASK_OFF(core)		(0x404 + (core << 4))
#define MBOX_INT_STATE_OFF(core)	(0x408 + (core << 4))
#define MBOX_INT_CLR_OFF(core)		(0x40C + (core << 4))
#define MBOX_INT_EN_OFF(core)		(0x500 + (core << 4))
#define MBOX_INT_DIS_OFF(core)		(0x504 + (core << 4))

#define HI6220_ACPU			(0x0)
#define HI6220_MCU			(0x2)

struct hi6220_mbox_dev {
	unsigned int index;

	unsigned int dir;
	unsigned int peer_core;

	unsigned int remote_irq;
	unsigned int local_irq;

	void __iomem *slot;
	struct hi6220_mbox *mbox;
};

struct hi6220_mbox {
	struct device *dev;

	unsigned int irq;
	bool tx_irq_mode;
	int irq_map[MBOX_MAX_CHANS];

	void __iomem *ipc;
	void __iomem *buf;

	struct hi6220_mbox_dev *mdevs;

	unsigned int chan_num;
	struct mbox_chan *chan;
	struct mbox_controller controller;
};

struct hisi_mbox_data {
	unsigned int chan_num;
	bool tx_irq_mode;
	struct hi6220_mbox_dev mdevs[MBOX_MAX_CHANS];
};

static u32 hi6220_mbox_read_data(struct hi6220_mbox_dev *mdev, int index)
{
	return readl(mdev->slot + MBOX_DATA_OFF(index));
}

static void hi6220_mbox_write_data(struct hi6220_mbox_dev *mdev,
				   int index, u32 data)
{
	writel(data, mdev->slot + MBOX_DATA_OFF(index));
}

static void hi6220_mbox_set_status(struct hi6220_mbox_dev *mdev, u32 val)
{
	u32 status;

	status = readl(mdev->slot + MBOX_MODE_OFF);
	status &= ~MBOX_STATUS_MASK;
	status |= val;
	writel(status, mdev->slot + MBOX_MODE_OFF);
}

static void hi6220_mbox_set_mode(struct hi6220_mbox_dev *mdev, u32 val)
{
	u32 mode;

	mode = readl(mdev->slot + MBOX_MODE_OFF);
	mode &= ~MBOX_ACK_CONFIG_MASK;
	mode |= val;
	writel(mode, mdev->slot + MBOX_MODE_OFF);
}

static void hi6220_mbox_set_target_irq(struct hi6220_mbox *mbox,
				       int target, int irq_num)
{
	writel(1 << irq_num, mbox->ipc + MBOX_INT_RAW_OFF(target));
}

static unsigned int hi6220_mbox_get_state(struct hi6220_mbox *mbox)
{
	return readl(mbox->ipc + MBOX_INT_STATE_OFF(HI6220_ACPU));
}

static void hi6220_mbox_clear_irq(struct hi6220_mbox *mbox, int irq_num)
{
	writel(1 << irq_num, mbox->ipc + MBOX_INT_CLR_OFF(HI6220_ACPU));
}

static void hi6220_mbox_enable_irq(struct hi6220_mbox *mbox, int irq_num)
{
	writel(1 << irq_num, mbox->ipc + MBOX_INT_EN_OFF(HI6220_ACPU));
}

static void hi6220_mbox_disable_irq(struct hi6220_mbox *mbox, int irq_num)
{
	writel(1 << irq_num, mbox->ipc + MBOX_INT_DIS_OFF(HI6220_ACPU));
}

static void hi6220_mbox_ipc_init(struct hi6220_mbox *mbox)
{
	/* mask and clear all interrupt vectors */
	writel(0x0, mbox->ipc + MBOX_INT_MASK_OFF(HI6220_ACPU));
	writel(0xFFFFFFFF, mbox->ipc + MBOX_INT_CLR_OFF(HI6220_ACPU));
}

static bool hi6220_mbox_last_tx_done(struct mbox_chan *chan)
{
	struct hi6220_mbox_dev *mdev = chan->con_priv;
	struct hi6220_mbox *mbox = mdev->mbox;

	/* Only set idle state for polling mode */
	BUG_ON(mbox->tx_irq_mode);

	hi6220_mbox_set_status(mdev, MBOX_STATUS_IDLE);
	return 1;
}

static int hi6220_mbox_send_data(struct mbox_chan *chan, void *data)
{
	struct hi6220_mbox_dev *mdev = chan->con_priv;
	struct hi6220_mbox *mbox = mdev->mbox;
	u32 *msg = data;
	int i;

	hi6220_mbox_set_status(mdev, MBOX_STATUS_TX);

	if (mbox->tx_irq_mode)
		hi6220_mbox_set_mode(mdev, MBOX_ACK_IRQ);
	else
		hi6220_mbox_set_mode(mdev, MBOX_ACK_AUTOMATIC);

	for (i = 0; i < 8; i++)
		hi6220_mbox_write_data(mdev, i, msg[i]);

	hi6220_mbox_set_target_irq(mbox, mdev->peer_core, mdev->remote_irq);
	return 0;
}

static int hi6220_mbox_startup(struct mbox_chan *chan)
{
	struct hi6220_mbox_dev *mdev = chan->con_priv;
	struct hi6220_mbox *mbox = mdev->mbox;

	mbox->irq_map[mdev->local_irq] = mdev->index;
	hi6220_mbox_enable_irq(mbox, mdev->local_irq);
	return 0;
}

static void hi6220_mbox_shutdown(struct mbox_chan *chan)
{

	struct hi6220_mbox_dev *mdev = chan->con_priv;
	struct hi6220_mbox *mbox = mdev->mbox;

	hi6220_mbox_disable_irq(mbox, mdev->local_irq);
	mbox->irq_map[mdev->local_irq] = -1;
}

static irqreturn_t hi6220_mbox_interrupt(int irq, void *p)
{
	struct hi6220_mbox *mbox = p;
	struct hi6220_mbox_dev *mdev;
	struct mbox_chan *chan;
	unsigned int state;
	unsigned int intr_bit;
	u32 data[8];
	int idx;

	state = hi6220_mbox_get_state(mbox);
	if (!state) {
		dev_warn(mbox->dev, "%s: sprious interrupt\n",
			 __func__);
		return IRQ_HANDLED;
	}

	while (state) {
		intr_bit = __ffs(state);
		state &= (state - 1);

		hi6220_mbox_clear_irq(mbox, intr_bit);

		idx = mbox->irq_map[intr_bit];
		if (idx == -1) {
			dev_warn(mbox->dev, "%s: un-expected vector %d\n",
				 __func__, idx);
			continue;
		}

		chan = &mbox->chan[idx];
		mdev = &mbox->mdevs[idx];

		if (mdev->dir == MBOX_TX) {
			hi6220_mbox_set_status(mdev, MBOX_STATUS_IDLE);
			mbox_chan_txdone(chan, 0);
		} else {
			int i;

			/* copy data */
			for (i = 0; i < 8; i++)
				data[i] = hi6220_mbox_read_data(mdev, i);
			mbox_chan_received_data(chan, (void *)data);
		}
	}

	return IRQ_HANDLED;
}

static struct mbox_chan_ops hi6220_mbox_ops = {
	.send_data    = hi6220_mbox_send_data,
	.startup      = hi6220_mbox_startup,
	.shutdown     = hi6220_mbox_shutdown,
	.last_tx_done = hi6220_mbox_last_tx_done,
};

/*
 * For Hi6220, enable two channels with irq mode;
 * channel 0: send message from MCU to ACPU;
 * channel 1: send message from ACPU to MCU.
 */
static const struct hisi_mbox_data hi6220_mbox_data = {
	.chan_num = 2,
	.tx_irq_mode = true,

	.mdevs[0] = { 0, MBOX_RX, HI6220_MCU, 1, 10, },
	.mdevs[1] = { 1, MBOX_TX, HI6220_MCU, 0, 11, },
};

static const struct of_device_id hi6220_mbox_of_match[] = {
	{
		.compatible = "hisilicon,hi6220-mbox",
		.data       = &hi6220_mbox_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, hi6220_mbox_of_match);

static int hi6220_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct device_node *np;
	const struct of_device_id *mbox_id;
	const struct hisi_mbox_data *data;
	struct hi6220_mbox *mbox;
	int i, err;

	np = of_find_matching_node_and_match(NULL, hi6220_mbox_of_match,
					     &mbox_id);
	if (!np)
		return -ENODEV;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	data = mbox_id->data;
	mbox->dev = dev;
	mbox->chan_num = data->chan_num;
	mbox->tx_irq_mode = data->tx_irq_mode;

	mbox->mdevs = devm_kzalloc(dev,
		mbox->chan_num * sizeof(struct hi6220_mbox_dev), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	memcpy(mbox->mdevs, data->mdevs,
			mbox->chan_num * sizeof(struct hi6220_mbox_dev));

	mbox->chan = devm_kzalloc(dev,
		mbox->chan_num * sizeof(struct mbox_chan), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	err = devm_request_irq(dev, irq_of_parse_and_map(dev->of_node, 0),
			hi6220_mbox_interrupt, 0, dev_name(dev), mbox);
	if (err) {
		dev_err(dev, "Failed to register a mailbox IRQ handler: %d\n",
			err);
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox->ipc = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox->ipc)) {
		dev_err(dev, "ioremap ipc failed\n");
		return PTR_ERR(mbox->ipc);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mbox->buf = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox->buf)) {
		dev_err(dev, "ioremap buffer failed\n");
		return PTR_ERR(mbox->buf);
	}

	for (i = 0; i < mbox->chan_num; i++) {
		mbox->irq_map[i] = -1;
		mbox->chan[i].con_priv = &mbox->mdevs[i];
		mbox->mdevs[i].index = i;
		mbox->mdevs[i].mbox = mbox;
		mbox->mdevs[i].slot = mbox->buf + (i << 6);
	}

	mbox->controller.dev = dev;
	mbox->controller.chans = &mbox->chan[0];
	mbox->controller.num_chans = mbox->chan_num;
	mbox->controller.ops = &hi6220_mbox_ops;

	if (mbox->tx_irq_mode)
		mbox->controller.txdone_irq = true;
	else {
		mbox->controller.txdone_poll = true;
		mbox->controller.txpoll_period = 5;
	}

	err = mbox_controller_register(&mbox->controller);
	if (err) {
		dev_err(dev, "Failed to register mailboxes %d\n", err);
		return err;
	}

	hi6220_mbox_ipc_init(mbox);
	platform_set_drvdata(pdev, mbox);
	dev_info(dev, "Mailbox enabled\n");
	return 0;
}

static int hi6220_mbox_remove(struct platform_device *pdev)
{
	struct hi6220_mbox *mbox = platform_get_drvdata(pdev);

	mbox_controller_unregister(&mbox->controller);
	return 0;
}

static struct platform_driver hi6220_mbox_driver = {
	.driver = {
		.name = "hi6220-mbox",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(hi6220_mbox_of_match),
	},
	.probe	= hi6220_mbox_probe,
	.remove	= hi6220_mbox_remove,
};

static int __init hi6220_mbox_init(void)
{
	return platform_driver_register(&hi6220_mbox_driver);
}
device_initcall(hi6220_mbox_init);

static void __exit hi6220_mbox_exit(void)
{
	platform_driver_unregister(&hi6220_mbox_driver);
}
module_exit(hi6220_mbox_exit);

MODULE_AUTHOR("Leo Yan <leo.yan@linaro.org>");
MODULE_DESCRIPTION("Hi6220 mailbox IPC driver");
MODULE_LICENSE("GPL v2");
