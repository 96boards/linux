/*
 * Copyright (c) 2014 Linaro Ltd.
 * Copyright (c) 2014 Hisilicon Limited.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/regulator/consumer.h>

#define SC_PERIPH_CTRL4			0x00c

#define PERIPH_CTRL4_PICO_SIDDQ		BIT(6)
#define PERIPH_CTRL4_PICO_OGDISABLE	BIT(8)
#define PERIPH_CTRL4_PICO_VBUSVLDEXT	BIT(10)
#define PERIPH_CTRL4_PICO_VBUSVLDEXTSEL	BIT(11)
#define PERIPH_CTRL4_OTG_PHY_SEL	BIT(21)

#define SC_PERIPH_CTRL5			0x010

#define PERIPH_CTRL5_USBOTG_RES_SEL	BIT(3)
#define PERIPH_CTRL5_PICOPHY_ACAENB	BIT(4)
#define PERIPH_CTRL5_PICOPHY_BC_MODE	BIT(5)
#define PERIPH_CTRL5_PICOPHY_CHRGSEL	BIT(6)
#define PERIPH_CTRL5_PICOPHY_VDATSRCEND	BIT(7)
#define PERIPH_CTRL5_PICOPHY_VDATDETENB	BIT(8)
#define PERIPH_CTRL5_PICOPHY_DCDENB	BIT(9)
#define PERIPH_CTRL5_PICOPHY_IDDIG	BIT(10)

#define SC_PERIPH_CTRL8			0x018

#define SC_PERIPH_RSTDIS0		0x304

#define PERIPH_RSTDIS0_MMC0		BIT(0)
#define PERIPH_RSTDIS0_MMC1		BIT(1)
#define PERIPH_RSTDIS0_MMC2		BIT(2)
#define PERIPH_RSTDIS0_NANDC		BIT(3)
#define PERIPH_RSTDIS0_USBOTG_BUS	BIT(4)
#define PERIPH_RSTDIS0_POR_PICOPHY	BIT(5)
#define PERIPH_RSTDIS0_USBOTG		BIT(6)
#define PERIPH_RSTDIS0_USBOTG_32K	BIT(7)

#define to_hisi_phy(p) container_of((p), struct hisi_priv, phy)

enum usb_mode {
	USB_EMPTY,
	GADGET_DEVICE,
	OTG_HOST,
	USB_HOST,
};

struct hisi_priv {
	struct usb_phy phy;
	struct delayed_work work;
	struct device *dev;
	void __iomem *base;
	struct regmap *reg;
	struct clk *clk;
	int gpio_vbus_det;
	int gpio_id_det;
	enum usb_mode mode;
};

static void hisi_start_periphrals(struct hisi_priv *priv, enum usb_mode mode)
{
	struct usb_otg *otg = priv->phy.otg;

	if (!otg->gadget)
		return;

	if (mode == GADGET_DEVICE)
		usb_gadget_connect(otg->gadget);
	else if (mode == USB_EMPTY)
		usb_gadget_disconnect(otg->gadget);
}

static int hisi_phy_setup(struct hisi_priv *priv, enum usb_mode mode)
{
	u32 val, mask;
	int ret;

	if (priv->mode == mode)
		return 0;

	if (mode == USB_EMPTY) {
		val = PERIPH_CTRL4_PICO_SIDDQ;
		mask = val;
		ret = regmap_update_bits(priv->reg, SC_PERIPH_CTRL4, mask, val);
		if (ret)
			goto out;

		ret = regmap_read(priv->reg, SC_PERIPH_CTRL4, &val);

		val = PERIPH_RSTDIS0_USBOTG_BUS | PERIPH_RSTDIS0_POR_PICOPHY |
			PERIPH_RSTDIS0_USBOTG | PERIPH_RSTDIS0_USBOTG_32K;
		mask = val;
		ret = regmap_update_bits(priv->reg, 0x300, mask, val);
		if (ret)
			goto out;
	} else {

		val = PERIPH_RSTDIS0_USBOTG_BUS | PERIPH_RSTDIS0_POR_PICOPHY |
			PERIPH_RSTDIS0_USBOTG | PERIPH_RSTDIS0_USBOTG_32K;
		mask = val;
		ret = regmap_update_bits(priv->reg, SC_PERIPH_RSTDIS0, mask, val);
		if (ret)
			goto out;

		ret = regmap_read(priv->reg, SC_PERIPH_CTRL5, &val);
		val = PERIPH_CTRL5_USBOTG_RES_SEL | PERIPH_CTRL5_PICOPHY_ACAENB;
		val |= 0x300;
		mask = val | PERIPH_CTRL5_PICOPHY_BC_MODE;
		ret = regmap_update_bits(priv->reg, SC_PERIPH_CTRL5, mask, val);
		if (ret)
			goto out;

		val =  PERIPH_CTRL4_PICO_VBUSVLDEXT | PERIPH_CTRL4_PICO_VBUSVLDEXTSEL |
			PERIPH_CTRL4_OTG_PHY_SEL;
		mask = val | PERIPH_CTRL4_PICO_SIDDQ | PERIPH_CTRL4_PICO_OGDISABLE;
		ret = regmap_update_bits(priv->reg, SC_PERIPH_CTRL4, mask, val);
		if (ret)
			goto out;

		val = 0x7053348c;
		ret = regmap_write(priv->reg, SC_PERIPH_CTRL8, val);
		if (ret)
			goto out;
	}

	priv->mode = mode;
	return 0;
out:
        dev_err(priv->dev, "failed to setup phy\n");
	return ret;
}

static void hisi_detect_work(struct work_struct *work)
{
	struct hisi_priv *priv =
		container_of(work, struct hisi_priv, work.work);
	int id_det, vbus_det;
	enum usb_mode mode;

	if (!gpio_is_valid(priv->gpio_id_det) ||
	    !gpio_is_valid(priv->gpio_vbus_det))
		return;

	id_det = gpio_get_value_cansleep(priv->gpio_id_det);
	vbus_det = gpio_get_value_cansleep(priv->gpio_vbus_det);

	if (vbus_det == 0) {
		if (id_det == 1) {
			mode = GADGET_DEVICE;
			//hisi_phy_setup(priv, mode);
		} else {
			mode = OTG_HOST;
			//hisi_phy_setup(priv, mode);
		}
	} else {
		mode = USB_EMPTY;
	//	hisi_phy_setup(priv, mode);
	//	hisi_phy_setup(priv, OTG_HOST);
	}
//	hisi_phy_setup(priv, mode);
	hisi_start_periphrals(priv, mode);
}

static irqreturn_t hiusb_gpio_intr(int irq, void *data)
{
	struct hisi_priv *priv = (struct hisi_priv *)data;

	/* add debounce time */
	schedule_delayed_work(&priv->work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static int hisi_phy_init(struct usb_phy *phy)
{
	return 0;
}

static void hisi_phy_shutdown(struct usb_phy *phy)
{
}

static int hisi_phy_suspend(struct usb_phy *x, int suspend)
{
	return 0;
}

static int hisi_phy_on_connect(struct usb_phy *phy,
		enum usb_device_speed speed)
{
	dev_dbg(phy->dev, "%s speed device has connected\n",
		(speed == USB_SPEED_HIGH) ? "high" : "non-high");
	return 0;
}

static int hisi_phy_on_disconnect(struct usb_phy *phy,
		enum usb_device_speed speed)
{
	dev_dbg(phy->dev, "%s speed device has disconnected\n",
		(speed == USB_SPEED_HIGH) ? "high" : "non-high");

	return 0;
}

static int mv_otg_set_host(struct usb_otg *otg,
			   struct usb_bus *host)
{
	otg->host = host;
	return 0;
}

static int mv_otg_set_peripheral(struct usb_otg *otg,
				 struct usb_gadget *gadget)
{
	struct hisi_priv *priv;

	priv = container_of(otg->phy, struct hisi_priv, phy);

	otg->gadget = gadget;
	return 0;
}

static int hisi_phy_probe(struct platform_device *pdev)
{
	struct hisi_priv *priv;
	struct usb_otg *otg;
	struct device_node *np = pdev->dev.of_node;
	struct regulator *reg_vhub;
	int ret, irq;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg)
		return -ENOMEM;

	reg_vhub = devm_regulator_get(&pdev->dev, "vhub");
	if (!IS_ERR(reg_vhub)) {
		ret = regulator_enable(reg_vhub);
		if (ret) {
			dev_err(&pdev->dev, "Failed to enable vhub regulator: %d\n", ret);
			return -ENODEV;
		}
	}

	priv->dev = &pdev->dev;
	priv->phy.dev = priv->dev;
	priv->phy.otg = otg;
	priv->phy.label = "hisi";
	priv->phy.init = hisi_phy_init;
	priv->phy.shutdown = hisi_phy_shutdown;
	priv->phy.set_suspend = hisi_phy_suspend;
	priv->phy.notify_connect = hisi_phy_on_connect;
	priv->phy.notify_disconnect = hisi_phy_on_disconnect;
	platform_set_drvdata(pdev, priv);

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk))
		return PTR_ERR(priv->clk);
	clk_prepare_enable(priv->clk);

	otg->set_host = mv_otg_set_host;
	otg->set_peripheral = mv_otg_set_peripheral;

	priv->gpio_vbus_det = of_get_named_gpio(np, "huawei,gpio_vbus_det", 0);
	if (priv->gpio_vbus_det== -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!gpio_is_valid(priv->gpio_vbus_det)) {
		dev_err(&pdev->dev, "invalid gpio pins %d\n", priv->gpio_vbus_det);
		return -ENODEV;
	}

	priv->gpio_id_det = of_get_named_gpio(np, "huawei,gpio_id_det", 0);
	if (priv->gpio_id_det== -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!gpio_is_valid(priv->gpio_id_det)) {
		dev_err(&pdev->dev, "invalid gpio pins %d\n", priv->gpio_id_det);
		return -ENODEV;
	}

	priv->reg = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
					"hisilicon,peripheral-syscon");
	if (IS_ERR(priv->reg))
		priv->reg = NULL;

	INIT_DELAYED_WORK(&priv->work, hisi_detect_work);

	ret = gpio_request(priv->gpio_vbus_det, "gpio_vbus_det");
        if (ret < 0) {
		dev_err(&pdev->dev, "gpio request failed for otg-int-gpio\n");
		return ret;
        }

	ret = gpio_request(priv->gpio_id_det, "gpio_id_det");
        if (ret < 0) {
		dev_err(&pdev->dev, "gpio request failed for otg-int-gpio\n");
		return ret;
        }

        gpio_direction_input(priv->gpio_vbus_det);
        gpio_direction_input(priv->gpio_id_det);
	irq = gpio_to_irq(priv->gpio_vbus_det);
	ret = devm_request_irq(&pdev->dev, gpio_to_irq(priv->gpio_vbus_det),
			       hiusb_gpio_intr, IRQF_NO_SUSPEND |
			       IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			       "vbus_gpio_intr", priv);
        if (ret) {
		dev_err(&pdev->dev, "request gpio irq failed.\n");
		goto err_irq;
        }
	hisi_phy_setup(priv, GADGET_DEVICE);

	ret = usb_add_phy(&priv->phy, USB_PHY_TYPE_USB2);
	if (ret) {
		dev_err(&pdev->dev, "Can't register transceiver\n");
		goto err_irq;
	}

	return 0;

err_irq:
	gpio_free(priv->gpio_vbus_det);
	gpio_free(priv->gpio_id_det);
	return ret;
}

static int hisi_phy_remove(struct platform_device *pdev)
{
	struct hisi_priv *priv = platform_get_drvdata(pdev);

	clk_disable_unprepare(priv->clk);
	gpio_free(priv->gpio_vbus_det);
	gpio_free(priv->gpio_id_det);
	return 0;
}

static const struct of_device_id hisi_phy_of_match[] = {
	{.compatible = "hisilicon,hi6220-usb-phy",},
	{ },
};
MODULE_DEVICE_TABLE(of, hisi_phy_of_match);

static struct platform_driver hisi_phy_driver = {
	.probe	= hisi_phy_probe,
	.remove	= hisi_phy_remove,
	.driver = {
		.name	= "hisi-usb-phy",
		.of_match_table	= hisi_phy_of_match,
	}
};
module_platform_driver(hisi_phy_driver);

MODULE_DESCRIPTION("HISILICON HISI USB PHY driver");
MODULE_ALIAS("platform:hisi-usb-phy");
MODULE_LICENSE("GPL");
