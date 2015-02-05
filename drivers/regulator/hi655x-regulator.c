/*
 * Hisilicon HI655X PMIC regulator driver
 *
 * Copyright (c) 2015 Hisilicon Co. Ltd.
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

#include <linux/slab.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/regulator/hi655x-regulator.h>
#include <linux/mfd/hi655x-pmic.h>

#define FORMAT_SSI_PMUREG(base, reg) (*(volatile unsigned int *)((char *)base + (reg)))
#define BSP_REG_SETBITS(base, reg, pos, bits, val) \
			((FORMAT_SSI_PMUREG(base, reg) = (FORMAT_SSI_PMUREG(base, reg) \
			& (~((((unsigned int)1 << (bits)) - 1) << (pos)))) \
			| ((unsigned int)((val) & (((unsigned int)1 << (bits)) - 1)) << (pos))))

#define BSP_REG_GETBITS(base, reg, pos, bits) \
			(((FORMAT_SSI_PMUREG(base, reg) >> (pos)) \
			&(((unsigned int)1 << (bits)) - 1)))

struct hi655x_regulator {
	int status;                             /* this property in child  node */
	unsigned int off_on_delay;              /* this property in parent node */
	enum hi655x_regulator_type type;        /* this property in child  node */
	int regulator_id;

	/* this property must be unify which is in child node */
	struct hi655x_regulator_ctrl_regs ctrl_regs;
	struct hi655x_regulator_ctrl_data ctrl_data;

	struct hi655x_regulator_vset_regs vset_regs;
	struct hi655x_regulator_vset_data vset_data;
	unsigned int vol_numb;
	unsigned int *vset_table;
	struct regulator_desc rdesc;
	int (*dt_parse)(struct hi655x_regulator*, struct platform_device*);
};

static int hi655x_regulator_pmic_is_enabled(struct regulator_dev *rdev)
{
	int ret = 0;
	unsigned char value_u8 = 0;

	struct hi655x_regulator *sreg = rdev_get_drvdata(rdev);
	struct hi655x_regulator_ctrl_regs *ctrl_regs = &(sreg->ctrl_regs);
	struct hi655x_regulator_ctrl_data *ctrl_data = &(sreg->ctrl_data);

	/*
	 * regulator is all set,but the pmu is only subset.
	 * maybe this "buck"/"ldo"/"lvs" is not contrl by a core.
	 * and in regulator have a "status" member ("okey" or "disable").
	 */
	value_u8 = hi655x_pmic_reg_read(ctrl_regs->status_reg);
	ret = ((int)BSP_REG_GETBITS(&value_u8, 0, ctrl_data->shift, ctrl_data->mask) & 0xff);

	return ret;
}

static int hi655x_regulator_pmic_enable(struct regulator_dev *rdev)
{
	int ret = 0;
	unsigned char value_u8 = 0;
	unsigned int value_u32 = 0;
	struct hi655x_regulator *sreg = rdev_get_drvdata(rdev);
	struct hi655x_regulator_ctrl_regs *ctrl_regs = &(sreg->ctrl_regs);
	struct hi655x_regulator_ctrl_data *ctrl_data = &(sreg->ctrl_data);

	BSP_REG_SETBITS(&value_u32, 0, ctrl_data->shift, ctrl_data->mask, 0x1);
	value_u8  = (unsigned char)value_u32;
	hi655x_pmic_reg_write(ctrl_regs->enable_reg, value_u8);
	udelay(sreg->off_on_delay);

	return ret;
}

static int hi655x_regulator_pmic_disable(struct regulator_dev *rdev)
{
	int ret = 0;
	int flag = 1;
	unsigned char value_u8 = 0;
	unsigned int value_u32 = 0;

	struct hi655x_regulator *sreg = rdev_get_drvdata(rdev);
	struct hi655x_regulator_ctrl_regs *ctrl_regs = &(sreg->ctrl_regs);
	struct hi655x_regulator_ctrl_data *ctrl_data = &(sreg->ctrl_data);

	/*
	 * regulator is all set,but the pmu is only subset.
	 * maybe this "buck"/"ldo"/"lvs" is not contrl by a core.
	 * and in regulator have a "status" member (okey or disable).
	 * maybe we can del some regulator which is not contrl by core.
	 */
	if (sreg->type == PMIC_BOOST_TYPE)
		flag = 0;

	BSP_REG_SETBITS(&value_u32, 0, ctrl_data->shift, ctrl_data->mask, flag);
	value_u8  = (unsigned char)value_u32;
	hi655x_pmic_reg_write(ctrl_regs->disable_reg, value_u8);

	return ret;
}

static int hi655x_regulator_pmic_list_voltage_linear(struct regulator_dev *rdev,
				  unsigned int selector)
{
	struct hi655x_regulator *sreg = rdev_get_drvdata(rdev);
	/*
	 * regulator is all set,but the pmu is only subset.
	 * maybe this "buck"/"ldo"/"lvs" is not contrl by a core.
	 * and in regulator have a "status" member (okey or disable).
	 * maybe we can del some regulator which is not contrl by core.
	 * we will return min_uV
	 */
	if (sreg->type == PMIC_LVS_TYPE)
		return 900000;

	if (selector >= sreg->vol_numb) {
		printk("selector err %s %d  \n", __func__, __LINE__);
		return -1;
	}

	return sreg->vset_table[selector];
}

static int hi655x_regulator_pmic_get_voltage(struct regulator_dev *rdev)
{
	int index = 0;
	unsigned int value_u32 = 0;
	struct hi655x_regulator *sreg = rdev_get_drvdata(rdev);
	struct hi655x_regulator_vset_regs *vset_regs = &(sreg->vset_regs);
	struct hi655x_regulator_vset_data *vset_data = &(sreg->vset_data);

	if (sreg->type == PMIC_LVS_TYPE)
		return 900000;

	value_u32 = (unsigned int)hi655x_pmic_reg_read(vset_regs->vset_reg) & 0xff;
	index = ((unsigned int)BSP_REG_GETBITS(&value_u32, 0, vset_data->shift, vset_data->mask) & 0xff);

	return sreg->vset_table[index];
}

static int hi655x_regulator_pmic_set_voltage(struct regulator_dev *rdev,
				int min_uV, int max_uV, unsigned *selector)
{
	int i = 0;
	int ret = 0;
	int vol = 0;
	unsigned char value_u8 = 0;
	unsigned int value_u32 = 0;

	struct hi655x_regulator *sreg = rdev_get_drvdata(rdev);
	struct hi655x_regulator_vset_regs *vset_regs = &(sreg->vset_regs);
	struct hi655x_regulator_vset_data *vset_data = &(sreg->vset_data);

	if (sreg->type == PMIC_LVS_TYPE)
		return 0;
	/*
	 * search the matched vol and get its index
	 */
	for (i = 0; i < sreg->vol_numb; i++) {
		vol = sreg->vset_table[i];

		if ((vol >= min_uV) && (vol <= max_uV))
			break;
	}

	if (i == sreg->vol_numb)
		return -1;

	value_u32 = (unsigned int)hi655x_pmic_reg_read(vset_regs->vset_reg) & 0xff;
	BSP_REG_SETBITS(&value_u32, 0, vset_data->shift, vset_data->mask, i);
	value_u8 = (unsigned char)value_u32;
	hi655x_pmic_reg_write(vset_regs->vset_reg, value_u8);

	*selector = i;

	return ret;
}

static unsigned int hi655x_regulator_pmic_get_mode(struct regulator_dev *rdev)
{
	return REGULATOR_MODE_NORMAL;
}

static int hi655x_regulator_pmic_set_mode(struct regulator_dev *rdev,
						unsigned int mode)
{
	return 0;
}

static unsigned int hi655x_regulator_pmic_get_optimum_mode(struct regulator_dev *rdev,
			int input_uV, int output_uV, int load_uA)
{
	return REGULATOR_MODE_NORMAL;
}

static struct regulator_ops hi655x_regulator_pmic_rops = {
	.is_enabled = hi655x_regulator_pmic_is_enabled,
	.enable = hi655x_regulator_pmic_enable,
	.disable = hi655x_regulator_pmic_disable,
	.list_voltage = hi655x_regulator_pmic_list_voltage_linear,
	.get_voltage = hi655x_regulator_pmic_get_voltage,
	.set_voltage = hi655x_regulator_pmic_set_voltage,
	.get_mode = hi655x_regulator_pmic_get_mode,
	.set_mode = hi655x_regulator_pmic_set_mode,
	.get_optimum_mode = hi655x_regulator_pmic_get_optimum_mode,
};

static int hi655x_regualtor_dt_parse_common(struct hi655x_regulator *sreg,
				struct platform_device *pdev)
{
	return 0;
}

static int hi655x_regualtor_pmic_dt_parse(struct hi655x_regulator *sreg,
					struct platform_device *pdev)
{
	int ret;
	ret = hi655x_regualtor_dt_parse_common(sreg, pdev);

	return ret;
}

static const struct hi655x_regulator hi655x_regulator_pmic = {
	.rdesc = {
		.ops = &hi655x_regulator_pmic_rops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	.dt_parse = hi655x_regualtor_pmic_dt_parse,
};

static struct of_device_id of_hi655x_regulator_match_tbl[] = {
	{
		.compatible = "hisilicon,hi6552-regulator-pmic",
		.data = &hi655x_regulator_pmic,
	},
	{ /* end */ }
};

static struct regulator_init_data *fake_of_get_regulator_init_data(struct device *dev,
				struct device_node *np)
{
	struct regulator_init_data *init_data = NULL;
	const __be32 *num_consumer_supplies = NULL;
	struct regulator_consumer_supply *consumer_supplies = NULL;
	int consumer_id = 0;

	init_data = devm_kzalloc(dev, sizeof(*init_data), GFP_KERNEL);
	if (!init_data)
		return NULL;

	init_data->supply_regulator = of_get_property(np, "hisilicon,regulator-supply", NULL);

	num_consumer_supplies = of_get_property(np, "hisilicon,num_consumer_supplies", NULL);
	if ((NULL == num_consumer_supplies) || (0 == *num_consumer_supplies)) {
		dev_warn(dev, "%s no consumer_supplies \n", __func__);
		return init_data;
	}

	init_data->num_consumer_supplies = be32_to_cpu(*num_consumer_supplies);
	init_data->consumer_supplies = (struct regulator_consumer_supply *)devm_kzalloc(dev,
					init_data->num_consumer_supplies * sizeof(struct regulator_consumer_supply), GFP_KERNEL);
	if (NULL == init_data->consumer_supplies) {
		dev_err(dev, "%s devm_kzalloc consumer_supplies err\n", __func__);
		return NULL;
	}

	consumer_supplies = init_data->consumer_supplies ;

	for (consumer_id = 0; consumer_id < init_data->num_consumer_supplies;
		consumer_id++, consumer_supplies++) {
		int ret = of_property_read_string_index(np, "hisilicon,consumer-supplies", consumer_id, &consumer_supplies->supply);
		if (ret)
			dev_err(dev, "\n %s of_property_read_string_index consumer-supplies err\n", np->name);
	}

	return init_data;
}

static int fake_of_get_regulator_constraint(struct regulation_constraints *constraints,
						struct device_node *np)
{
	const __be32 *min_uV, *max_uV;
	unsigned int *valid_modes_mask;
	unsigned int *valid_ops_mask;
	unsigned int *initial_mode;

	if (!np)
		return -1;

	if (!constraints)
		return -1;

	constraints->name = of_get_property(np, "regulator-name", NULL);

	min_uV = of_get_property(np, "regulator-min-microvolt", NULL);
	if (min_uV) {
		(constraints)->min_uV = be32_to_cpu(*min_uV);
		(constraints)->min_uA = be32_to_cpu(*min_uV);
	}

	max_uV = of_get_property(np, "regulator-max-microvolt", NULL);
	if (max_uV) {
		constraints->max_uV = be32_to_cpu(*max_uV);
		constraints->max_uA = be32_to_cpu(*max_uV);
	}

	valid_modes_mask = (unsigned int *)of_get_property(np, "hisilicon,valid-modes-mask", NULL);
	if (valid_modes_mask)
		constraints->valid_modes_mask = be32_to_cpu(*valid_modes_mask);

	valid_ops_mask = (unsigned int *)of_get_property(np, "hisilicon,valid-ops-mask", NULL);
	if (valid_ops_mask)
		constraints->valid_ops_mask = be32_to_cpu(*valid_ops_mask);

	initial_mode = (unsigned int *)of_get_property(np, "hisilicon,initial-mode", NULL);
	if (initial_mode)
		constraints->initial_mode = be32_to_cpu(*initial_mode);

	constraints->always_on = !!(of_find_property(np, "regulator-always-on", NULL));

	constraints->boot_on = !!(of_find_property(np, "regulator-boot-on", NULL));

	return 0;
}

static int fake_of_get_regulator_sreg(struct hi655x_regulator *sreg, struct device *dev,
						struct device_node *np)
{
	int *vol_numb;
	unsigned int *off_on_delay;
	enum hi655x_regulator_type *regulator_type;
	const char *status = NULL;
	unsigned int *vset_table = NULL;
	int *regulator_id;

	status = of_get_property(np, "hisilicon,regulator-status", NULL);
	if (status)
		sreg->status = !(strcmp(status, "okey"));

	regulator_type = (enum hi655x_regulator_type *)of_get_property(np, "hisilicon,regulator-type", NULL);
	if (regulator_type)
		sreg->type = be32_to_cpu(*regulator_type);

	off_on_delay = (unsigned int *)of_get_property(np, "hisilicon,off-on-delay", NULL);
	if (off_on_delay)
		sreg->off_on_delay = be32_to_cpu(*off_on_delay);

	of_property_read_u32_array(np, "hisilicon,ctrl-regs", (unsigned int *)(&sreg->ctrl_regs), 0x3);

	of_property_read_u32_array(np, "hisilicon,ctrl-data", (unsigned int *)(&sreg->ctrl_data), 0x2);

	of_property_read_u32_array(np, "hisilicon,vset-regs", (unsigned int *)(&sreg->vset_regs), 0x1);

	of_property_read_u32_array(np, "hisilicon,vset-data", (unsigned int *)(&sreg->vset_data), 0x2);

	vol_numb = (int *)of_get_property(np, "hisilicon,regulator-n-vol", NULL);
	if (vol_numb)
		sreg->vol_numb = be32_to_cpu(*vol_numb);

	regulator_id = (int *)of_get_property(np, "hisilicon,hisi-scharger-regulator-id", NULL);
	if (regulator_id)
		sreg->regulator_id =  be32_to_cpu(*regulator_id);

	vset_table = devm_kzalloc(dev, sreg->vol_numb * sizeof(int), GFP_KERNEL);
	if (!vset_table)
		return -1;

	of_property_read_u32_array(np, "hisilicon,vset-table", (unsigned int *)vset_table, sreg->vol_numb);
	sreg->vset_table = vset_table;

	return 0;
}

static int hi655x_regulator_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct regulator_dev *rdev = NULL;
	struct regulator_desc *rdesc = NULL;
	struct hi655x_regulator *sreg = NULL;
	struct regulator_init_data *initdata = NULL;
	const struct of_device_id *match = NULL;
	const struct hi655x_regulator *template = NULL;
	struct regulator_config config = { };

	/* to check which type of regulator this is */
	match = of_match_device(of_hi655x_regulator_match_tbl, &pdev->dev);

	if (NULL == match) {
		dev_err(dev, "of match hi655x regulator fail!\n\r");
		return -EINVAL;
	}

	/*tempdev is regulator device*/
	template = match->data;

	/*just for getting "std regulator node" value-key about constraint*/
	initdata = fake_of_get_regulator_init_data(dev, np);
	if (!initdata) {
		dev_err(dev, "get regulator init data error !\n");
		return -EINVAL;
	}

	ret = fake_of_get_regulator_constraint(&initdata->constraints, np);
	if (!!ret) {
		dev_err(dev, "get regulator constraint error !\n");
		return -EINVAL;
	}

	/* TODO:hi655x regulator supports two modes */
	sreg = kmemdup(template, sizeof(*sreg), GFP_KERNEL);
	if (!sreg)
		return -ENOMEM;

	if (0 != fake_of_get_regulator_sreg(sreg, dev, np)) {
		kfree(sreg);
		return -EINVAL;
	}

	rdesc = &sreg->rdesc;
	rdesc->n_voltages = sreg->vol_numb;
	rdesc->name = initdata->constraints.name;
	rdesc->id = sreg->regulator_id;
	rdesc->min_uV = initdata->constraints.min_uV;

	/* to parse device tree data for regulator specific */
	config.dev = &pdev->dev;
	config.init_data = initdata;
	config.driver_data = sreg;
	config.of_node = pdev->dev.of_node;

	/* register regulator */
	rdev = regulator_register(rdesc, &config);
	if (IS_ERR(rdev)) {
		dev_err(dev, "regulator failed to register %s\n", rdesc->name);
		ret = PTR_ERR(rdev);
		return -EINVAL;
	}

	platform_set_drvdata(pdev, rdev);

	return ret;
}

static int hi655x_regulator_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver hi655x_regulator_driver = {
	.driver = {
		.name	= "hi655x_regulator",
		.owner  = THIS_MODULE,
		.of_match_table = of_hi655x_regulator_match_tbl,
	},
	.probe	= hi655x_regulator_probe,
	.remove	= hi655x_regulator_remove,
};

static int __init hi655x_regulator_init(void)
{
	return platform_driver_register(&hi655x_regulator_driver);
}

static void __exit hi655x_regulator_exit(void)
{
	platform_driver_unregister(&hi655x_regulator_driver);
}

fs_initcall(hi655x_regulator_init);
module_exit(hi655x_regulator_exit);

MODULE_AUTHOR("Dongbin Yu <yudongbin@huawei.com>");
MODULE_DESCRIPTION("Hisilicon HI655X PMIC regulator driver");
MODULE_LICENSE("GPL v2");
