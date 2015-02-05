/*
 * Hisilicon Hi655x series PMIC driver
 *
 * Copyright (c) 2015 Hisilicon Co. Ltd
 *
 * Author:
 * 	Dongbin Yu <yudongbin@huawei.com>
 *	Bintian Wang <bintian.wang@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under  the terms of the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the License, or (at your
 * option) any later version.
 *
 */

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/hardirq.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/irqdomain.h>
#include <linux/mfd/hi655x-pmic.h>

static void __iomem *PMUSSI_BASE_ADDR;

#define PMUSSI_REG(addr) ((char *)PMUSSI_BASE_ADDR + ((addr) << 2))

#define DEBUG_PMIC_GPIO

struct hi655x_pmic {
	struct resource *res;
	struct device *dev;
	spinlock_t ssi_hw_lock;
	struct clk *clk;
	struct irq_domain *domain;
	int irq;
	int gpio;
	unsigned int irqs[HI655x_NR_IRQ];
	unsigned int ver;
};

static struct hi655x_pmic *pmic_dev;

unsigned char hi655x_pmic_reg_read (unsigned int addr)
{
	unsigned char val;
	val =  *(volatile unsigned char*)PMUSSI_REG(addr);
	return val;
}
EXPORT_SYMBOL(hi655x_pmic_reg_read);

void hi655x_pmic_reg_write (unsigned int addr, unsigned char val)
{
	*(volatile unsigned char*)PMUSSI_REG(addr) = val;
}
EXPORT_SYMBOL(hi655x_pmic_reg_write);

unsigned char hi655x_pmic_reg_read_ex (void *pmu_base, unsigned int addr)
{
	unsigned char val;
	val =  *(volatile unsigned char*)PMUSSI_REG_EX(pmu_base, addr);
	return val;
}
EXPORT_SYMBOL(hi655x_pmic_reg_read_ex);

void hi655x_pmic_reg_write_ex(void *pmu_base, unsigned int addr, unsigned char val)
{
	*(volatile unsigned char*)PMUSSI_REG_EX(pmu_base, addr) = val;
}
EXPORT_SYMBOL(hi655x_pmic_reg_write_ex);

static struct of_device_id of_hi655x_pmic_child_match_tbl[] = {
	{ .compatible = "hisilicon,hi6552-regulator-pmic", },
	{ .compatible = "hisilicon,hi6552-powerkey", },
	{ .compatible = "hisilicon,hi6552-usbvbus", },
	{ .compatible = "hisilicon,hi6552-coul", },
	{ .compatible = "hisilicon,hi6552-pmu-rtc", },
	{ .compatible = "hisilicon,hi6552-pmic-mntn", },
	{ /* end */ }
};

static struct of_device_id of_hi655x_pmic_match_tbl[] = {
	{ .compatible = "hisilicon,hi6552-pmic-driver", },
	{ /* end */ }
};

unsigned int hi655x_pmic_get_version(void)
{
	unsigned int uvalue = 0;
	uvalue = (unsigned int)hi655x_pmic_reg_read(HI655x_VER_REG);
	uvalue = uvalue & HI655x_REG_WIDTH;

	return uvalue;
}

static int hi655x_pmic_version_check(void)
{
	int ret = SSI_DEVICE_ERR;
	int ver = 0;

	ver = hi655x_pmic_get_version();
	if ((ver >= PMU_VER_START) && (ver <= PMU_VER_END))
		return SSI_DEVICE_OK;

	return ret;
}

static irqreturn_t hi655x_pmic_irq_handler(int irq, void *data)
{
	struct hi655x_pmic *pmic = (struct hi655x_pmic *)data;
	unsigned long pending;
	unsigned int ret = IRQ_NONE;
	int i, offset;

	for (i = 0; i < HI655x_IRQ_ARRAY; i++) {
		pending = hi655x_pmic_reg_read((i + HI655x_IRQ_STAT_BASE));
		pending &= HI655x_REG_WIDTH;
		if (pending != 0)
			pr_debug("pending[%d]=0x%lx\n\r", i, pending);

		/* clear pmic-sub-interrupt */
		hi655x_pmic_reg_write((i + HI655x_IRQ_STAT_BASE), pending);

		if (pending) {
			for_each_set_bit(offset, &pending, HI655x_BITS)
				generic_handle_irq(pmic->irqs[offset + i * HI655x_BITS]);
			ret = IRQ_HANDLED;
		}
	}

	return ret;
}

static void hi655x_pmic_irq_mask(struct irq_data *d)
{
	u32 data, offset;
	unsigned long pmic_spin_flag = 0;
	offset = ((irqd_to_hwirq(d) >> 3) + HI655x_IRQ_MASK_BASE);

	spin_lock_irqsave(&pmic_dev->ssi_hw_lock, pmic_spin_flag);
	data = hi655x_pmic_reg_read(offset);
	data |= (1 << (irqd_to_hwirq(d) & 0x07));
	hi655x_pmic_reg_write(offset, data);
	spin_unlock_irqrestore(&pmic_dev->ssi_hw_lock, pmic_spin_flag);
}

static void hi655x_pmic_irq_unmask(struct irq_data *d)
{
	u32 data, offset;
	unsigned long pmic_spin_flag = 0;
	offset = ((irqd_to_hwirq(d) >> 3) + HI655x_IRQ_MASK_BASE);

	spin_lock_irqsave(&pmic_dev->ssi_hw_lock, pmic_spin_flag);
	data = hi655x_pmic_reg_read(offset);
	data &= ~(1 << (irqd_to_hwirq(d) & 0x07));
	hi655x_pmic_reg_write(offset, data);
	spin_unlock_irqrestore(&pmic_dev->ssi_hw_lock, pmic_spin_flag);
}

static struct irq_chip hi655x_pmic_irqchip = {
	.name		= "hisi-hi655x-pmic-irqchip",
	.irq_mask	= hi655x_pmic_irq_mask,
	.irq_unmask	= hi655x_pmic_irq_unmask,
};

static int hi655x_pmic_irq_map(struct irq_domain *d, unsigned int virq,
			  irq_hw_number_t hw)
{
	struct hi655x_pmic *pmic = d->host_data;

	irq_set_chip_and_handler_name(virq, &hi655x_pmic_irqchip,
				      handle_simple_irq, "hisi-hi655x-pmic-irqchip");
	irq_set_chip_data(virq, pmic);
	irq_set_irq_type(virq, IRQ_TYPE_NONE);

	return 0;
}

static struct irq_domain_ops hi655x_domain_ops = {
	.map	= hi655x_pmic_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

static inline void hi655x_pmic_clear_int(void)
{
	int addr;

	for (addr = HI655x_IRQ_STAT_BASE; addr < (HI655x_IRQ_STAT_BASE + HI655x_IRQ_ARRAY); addr++)
		hi655x_pmic_reg_write(addr, HI655x_IRQ_CLR);
}

static inline void hi655x_pmic_mask_int(void)
{
	int addr;

	for (addr = HI655x_IRQ_MASK_BASE; addr < (HI655x_IRQ_MASK_BASE + HI655x_IRQ_ARRAY); addr++)
		hi655x_pmic_reg_write(addr, HI655x_IRQ_MASK);
}

static int hi655x_pmic_probe(struct platform_device *pdev)
{
	int i = 0;
	int ret = 0 ;
	int dev_stat = 0;
	unsigned int virq = 0;
	int pmu_on = 1;
	enum of_gpio_flags gpio_flags;
	struct device_node *gpio_np = NULL;

	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct hi655x_pmic *pmic = NULL;

	/*
	 * this is new feature in kernel 3.10
	 */
	pmic = devm_kzalloc(dev, sizeof(*pmic), GFP_KERNEL);
	if (!pmic) {
		printk("cannot allocate hi655x_pmic device info\n");
		return -ENOMEM;
	}
	pmic_dev = pmic;

	/* init spin lock */
	spin_lock_init(&pmic->ssi_hw_lock);

	pmic->res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pmic->res) {
		printk("platform_get_resource err\n");
		return -ENOENT;
	}
	if (!devm_request_mem_region(dev, pmic->res->start,
				     resource_size(pmic->res),
				     pdev->name)) {
		printk("cannot claim register memory\n");
		return -ENOMEM;
	}
	PMUSSI_BASE_ADDR = ioremap(pmic->res->start,
				  resource_size(pmic->res));
	if (!PMUSSI_BASE_ADDR) {
		printk("cannot map register memory\n");
		return -ENOMEM;
	}

	/* confirm the pmu version */
	pmic->ver = hi655x_pmic_get_version();
	if ((pmic->ver < PMU_VER_START) || (pmic->ver > PMU_VER_END)) {
		pr_err("it is wrong pmu version\n");
		pmu_on = 0;
	}

	hi655x_pmic_reg_write(0x1b5, 0xff);

#ifdef DEBUG_PMIC_GPIO
	/*
	 * must finish the gpio&irq is function
	 */
	gpio_np = of_parse_phandle(np, "pmu_irq_gpio", 0);
	if (!gpio_np) {
		dev_err(dev, "can't parse property\n");
		return -ENOENT;
	}
	pmic->gpio = of_get_gpio_flags(gpio_np, 0, &gpio_flags);
	if (pmic->gpio < 0) {
		dev_err(dev, "failed to of_get_gpio_flags %d\n", pmic->gpio);
		return pmic->gpio;
	}
	if (!gpio_is_valid(pmic->gpio)) {
		dev_err(dev, "it is invalid gpio %d\n", pmic->gpio);
		return -EINVAL;
	}

	ret = gpio_request_one(pmic->gpio, GPIOF_IN, "hi655x_pmic_irq");
	if (ret < 0) {
		pr_err("failed to request gpio %d, ret:%d\n", pmic->gpio, ret);
		return ret;
	}
	pmic->irq = gpio_to_irq(pmic->gpio);
#endif

	/* clear PMIC sub-interrupt */
	hi655x_pmic_clear_int();

	/* mask PMIC sub-interrupt */
	hi655x_pmic_mask_int();

	/* register irq domain */
	pmic->domain = irq_domain_add_simple(np, HI655x_NR_IRQ, 0,
					     &hi655x_domain_ops, pmic);
	if (!pmic->domain) {
		pr_err("in %s failed irq domain add simple!\n", __func__);
		ret = -ENODEV;
		return ret;
	}

	for (i = 0; i < HI655x_NR_IRQ; i++) {
		virq = irq_create_mapping(pmic->domain, i);
		if (0 == virq) {
			printk("Failed mapping hwirq\n");
			ret = -ENOSPC;
			return ret;
		}
		pmic->irqs[i] = virq;
	}

	/* Check the GPIO status is high */
	if (pmu_on) {
		ret = request_threaded_irq(pmic->irq, hi655x_pmic_irq_handler, NULL,
				IRQF_TRIGGER_LOW | IRQF_SHARED | IRQF_NO_SUSPEND,
				"hi655x-pmic-irq", pmic);
		if (ret < 0) {
			pr_err("*************could not claim pmic %d\n", ret);
			ret = -ENODEV;
			return ret;
		}
	}

	pmic->dev = dev;

	/* bind pmic to device */
	platform_set_drvdata(pdev, pmic);

	/* populate sub nodes */
	of_platform_populate(np, of_hi655x_pmic_child_match_tbl, NULL, dev);

	dev_stat = hi655x_pmic_version_check();

	return 0;
}

#ifdef CONFIG_PM
static int hi655x_pmic_suspend(struct platform_device *pdev, pm_message_t pm)
{
	return 0;
}

static int hi655x_pmic_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct platform_driver pmic_driver = {
	.driver	= {
		.name =	"hisi,hi655x-pmic",
		.owner = THIS_MODULE,
		.of_match_table = of_hi655x_pmic_match_tbl,
	},
	.probe  = hi655x_pmic_probe,
#ifdef CONFIG_PM
	.suspend = hi655x_pmic_suspend,
	.resume = hi655x_pmic_resume,
#endif
};

static int __init hi655x_pmic_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&pmic_driver);
	if (ret) {
		printk("%s: platform_driver_register failed %d\n",
				__func__, ret);
	}

	return ret;
}

static void __exit hi655x_pmic_exit(void)
{
	platform_driver_unregister(&pmic_driver);
}

module_init(hi655x_pmic_init);
module_exit(hi655x_pmic_exit);

MODULE_AUTHOR("Dongbin Yu <yudongbin@huawei.com>");
MODULE_DESCRIPTION("Hisilicon HI655x PMU SSI interface driver");
MODULE_LICENSE("GPL v2");
