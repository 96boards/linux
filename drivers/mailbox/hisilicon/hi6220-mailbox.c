/*
 * Hisilicon's Hi6220 mailbox low level driver
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

#include <linux/err.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "common.h"

#define HI6220_MBOX_CHAN_NUM			2
#define HI6220_MBOX_CHAN_SLOT_SIZE		(1 << 6)

/* Status & Mode Register */
#define HI6220_MBOX_MODE_REG			(0x0)

#define HI6220_MBOX_STATUS_MASK			(0xF << 4)
#define HI6220_MBOX_STATUS_IDLE			(0x1 << 4)
#define HI6220_MBOX_STATUS_TX			(0x2 << 4)
#define HI6220_MBOX_STATUS_RX			(0x4 << 4)
#define HI6220_MBOX_STATUS_ACK			(0x8 << 4)
#define HI6220_MBOX_ACK_CONFIG_MASK		(0x1 << 0)
#define HI6220_MBOX_ACK_AUTOMATIC		(0x1 << 0)
#define HI6220_MBOX_ACK_IRQ			(0x0 << 0)

/* Data Registers */
#define HI6220_MBOX_DATA_REG(i)			(0x4 + (i << 2))

/* ACPU Interrupt Register */
#define HI6220_MBOX_ACPU_INT_RAW_REG		(0x400)
#define HI6220_MBOX_ACPU_INT_MSK_REG		(0x404)
#define HI6220_MBOX_ACPU_INT_STAT_REG		(0x408)
#define HI6220_MBOX_ACPU_INT_CLR_REG		(0x40c)
#define HI6220_MBOX_ACPU_INT_ENA_REG		(0x500)
#define HI6220_MBOX_ACPU_INT_DIS_REG		(0x504)

/* MCU Interrupt Register */
#define HI6220_MBOX_MCU_INT_RAW_REG		(0x420)

/* Core Id */
#define HI6220_CORE_ACPU			(0x0)
#define HI6220_CORE_MCU				(0x2)

static u32 hi6220_mbox_get_status(struct hisi_mbox_chan *chan)
{
	u32 status;

	status = readl(chan->slot + HI6220_MBOX_MODE_REG);
	return status & HI6220_MBOX_STATUS_MASK;
}

static void hi6220_mbox_set_status(struct hisi_mbox_chan *chan, u32 val)
{
	u32 status;

	status = readl(chan->slot + HI6220_MBOX_MODE_REG);
	status &= ~HI6220_MBOX_STATUS_MASK;
	status |= val;
	writel(status, chan->slot + HI6220_MBOX_MODE_REG);
}

static void hi6220_mbox_set_mode(struct hisi_mbox_chan *chan, u32 val)
{
	u32 mode;

	mode = readl(chan->slot + HI6220_MBOX_MODE_REG);
	mode &= ~HI6220_MBOX_ACK_CONFIG_MASK;
	mode |= val;
	writel(mode, chan->slot + HI6220_MBOX_MODE_REG);
}

static void hi6220_mbox_clear_irq(struct hisi_mbox_chan *chan)
{
	struct hisi_mbox_hw *mbox_hw = chan->mbox_hw;
	int irq = chan->local_irq;

	writel(1 << irq, mbox_hw->ipc + HI6220_MBOX_ACPU_INT_CLR_REG);
}

static void hi6220_mbox_enable_irq(struct hisi_mbox_chan *chan)
{
	struct hisi_mbox_hw *mbox_hw = chan->mbox_hw;
	int irq = chan->local_irq;

	writel(1 << irq, mbox_hw->ipc + HI6220_MBOX_ACPU_INT_ENA_REG);
}

static void hi6220_mbox_disable_irq(struct hisi_mbox_chan *chan)
{
	struct hisi_mbox_hw *mbox_hw = chan->mbox_hw;
	int irq = chan->local_irq;

	writel(1 << irq, mbox_hw->ipc + HI6220_MBOX_ACPU_INT_DIS_REG);
}

static u32 hi6220_mbox_get_pending_irq(struct hisi_mbox_hw *mbox_hw)
{
	return readl(mbox_hw->ipc + HI6220_MBOX_ACPU_INT_STAT_REG);
}

static int hi6220_mbox_startup(struct hisi_mbox_chan *chan)
{
	hi6220_mbox_enable_irq(chan);
	return 0;
}

static void hi6220_mbox_shutdown(struct hisi_mbox_chan *chan)
{
	hi6220_mbox_disable_irq(chan);
}

static int hi6220_mbox_rx(struct hisi_mbox_chan *chan, void *msg, int len)
{
	int *buf = msg;
	int i;

	for (i = 0; i < (len >> 2); i++)
		buf[i] = readl(chan->slot + HI6220_MBOX_DATA_REG(i));

	/* clear IRQ source */
	hi6220_mbox_clear_irq(chan);
	hi6220_mbox_set_status(chan, HI6220_MBOX_STATUS_IDLE);
	return len;
}

static int hi6220_mbox_tx(struct hisi_mbox_chan *chan, void *msg, int len)
{
	struct hisi_mbox_hw *mbox_hw = chan->mbox_hw;
	int *buf = msg;
	int irq = chan->remote_irq, i;

	hi6220_mbox_set_status(chan, HI6220_MBOX_STATUS_TX);

	if (mbox_hw->tx_irq_mode)
		hi6220_mbox_set_mode(chan, HI6220_MBOX_ACK_IRQ);
	else
		hi6220_mbox_set_mode(chan, HI6220_MBOX_ACK_AUTOMATIC);

	for (i = 0; i < (len >> 2); i++)
		writel(buf[i], chan->slot + HI6220_MBOX_DATA_REG(i));

	/* trigger remote request */
	writel(1 << irq, mbox_hw->ipc + HI6220_MBOX_MCU_INT_RAW_REG);
	return 0;
}

static int hi6220_mbox_tx_is_done(struct hisi_mbox_chan *chan)
{
	int status;

	status = hi6220_mbox_get_status(chan);
	return (status == HI6220_MBOX_STATUS_IDLE);
}

static int hi6220_mbox_ack(struct hisi_mbox_chan *chan)
{
	hi6220_mbox_set_status(chan, HI6220_MBOX_STATUS_IDLE);
	return 0;
}

static struct hisi_mbox_ops hi6220_mbox_ops = {
	.startup	 = hi6220_mbox_startup,
	.shutdown	 = hi6220_mbox_shutdown,

	.clear_irq	 = hi6220_mbox_clear_irq,
	.enable_irq	 = hi6220_mbox_enable_irq,
	.disable_irq	 = hi6220_mbox_disable_irq,
	.get_pending_irq = hi6220_mbox_get_pending_irq,

	.rx		 = hi6220_mbox_rx,
	.tx		 = hi6220_mbox_tx,
	.tx_is_done	 = hi6220_mbox_tx_is_done,
	.ack		 = hi6220_mbox_ack,
};

static void hi6220_mbox_init_hw(struct hisi_mbox_hw *mbox_hw)
{
	struct hisi_mbox_chan init_data[HI6220_MBOX_CHAN_NUM] = {
		{ MBOX_RX, HI6220_CORE_MCU, 1, 10 },
		{ MBOX_TX, HI6220_CORE_MCU, 0, 11 },
	};
	struct hisi_mbox_chan *chan = mbox_hw->chan;
	int i;

	for (i = 0; i < HI6220_MBOX_CHAN_NUM; i++) {
		memcpy(&chan[i], &init_data[i], sizeof(*chan));
		chan[i].slot = mbox_hw->buf + HI6220_MBOX_CHAN_SLOT_SIZE;
		chan[i].mbox_hw = mbox_hw;
	}

	/* mask and clear all interrupt vectors */
	writel(0x0,  mbox_hw->ipc + HI6220_MBOX_ACPU_INT_MSK_REG);
	writel(~0x0, mbox_hw->ipc + HI6220_MBOX_ACPU_INT_CLR_REG);

	/* use interrupt for tx's ack */
	mbox_hw->tx_irq_mode = true;

	/* set low level ops */
	mbox_hw->ops = &hi6220_mbox_ops;
}

static const struct of_device_id hi6220_mbox_of_match[] = {
	{ .compatible = "hisilicon,hi6220-mbox", },
	{},
};
MODULE_DEVICE_TABLE(of, hi6220_mbox_of_match);

static int hi6220_mbox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct hisi_mbox_hw *mbox_hw;
	struct resource *res;
	int err;

	mbox_hw = devm_kzalloc(dev, sizeof(*mbox_hw), GFP_KERNEL);
	if (!mbox_hw)
		return -ENOMEM;

	mbox_hw->dev = dev;
	mbox_hw->chan_num = HI6220_MBOX_CHAN_NUM;
	mbox_hw->chan = devm_kzalloc(dev,
		mbox_hw->chan_num * sizeof(struct hisi_mbox_chan), GFP_KERNEL);
	if (!mbox_hw->chan)
		return -ENOMEM;

	mbox_hw->irq = platform_get_irq(pdev, 0);
	if (mbox_hw->irq < 0)
		return mbox_hw->irq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	mbox_hw->ipc = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox_hw->ipc)) {
		dev_err(dev, "ioremap ipc failed\n");
		return PTR_ERR(mbox_hw->ipc);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	mbox_hw->buf = devm_ioremap_resource(dev, res);
	if (IS_ERR(mbox_hw->buf)) {
		dev_err(dev, "ioremap buffer failed\n");
		return PTR_ERR(mbox_hw->buf);
	}

	hi6220_mbox_init_hw(mbox_hw);

	err = hisi_mbox_register(mbox_hw);
	if (err)
		return err;

	platform_set_drvdata(pdev, mbox_hw);
	dev_info(dev, "Mailbox enabled\n");
	return 0;
}

static int hi6220_mbox_remove(struct platform_device *pdev)
{
	struct hisi_mbox_hw *mbox_hw = platform_get_drvdata(pdev);

	hisi_mbox_unregister(mbox_hw);
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
core_initcall(hi6220_mbox_init);

static void __exit hi6220_mbox_exit(void)
{
	platform_driver_unregister(&hi6220_mbox_driver);
}
module_exit(hi6220_mbox_exit);

MODULE_AUTHOR("Leo Yan <leo.yan@linaro.org>");
MODULE_DESCRIPTION("Hi6220 mailbox driver");
MODULE_LICENSE("GPL v2");
