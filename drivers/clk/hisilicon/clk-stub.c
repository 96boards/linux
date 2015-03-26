/*
 * Hisilicon stub clock driver
 *
 * Copyright (c) 2015 Hisilicon Limited.
 * Copyright (c) 2015 Linaro Limited.
 *
 * Author: Leo Yan <leo.yan@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <asm/compiler.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <dt-bindings/clock/hisi,stub-clock.h>

/* CPU dynamic frequency scaling */
#define ACPU_DFS_FREQ_MAX		(0x1724)
#define ACPU_DFS_FLAG			(0x1AF4)
#define ACPU_DFS_FREQ_REQ		(0x1AF8)
#define ACPU_DFS_FREQ_LMT		(0x1AFC)

#define ACPU_DFS_LOCK_FLAG		(0xAEAEAEAE)

/* Multi-core communication */
#define MC_CORE_ACPU			0x2
#define MC_COM_CPU_RAW_INT_OFFSET(i)	(0x400 + (i << 4))
#define MC_COM_INT_ACPU_DFS		15

#define to_stub_clk(hw) container_of(hw, struct hisi_stub_clk, hw)

struct hisi_stub_clk {
	struct clk_hw	hw;

	/*
	 * hi6220:
	 *  - 0: A53; 1: A53;  2: gpu;  3: ddr;
	 */
	u32		id;
	u32		rate;
	spinlock_t	*lock;
};

static int initialized_stub_clk = 0;
static struct regmap *mc_map = NULL;
static struct regmap *dfs_map = NULL;

static unsigned int hisi_acpu_get_freq(void)
{
	unsigned int freq;

	regmap_read(dfs_map, ACPU_DFS_FREQ_REQ, &freq);
	return freq;
}

static int hisi_acpu_set_freq(unsigned int freq)
{
	/* set the frequency in sram */
	regmap_write(dfs_map, ACPU_DFS_FREQ_REQ, freq);

	/* send request to power controller */
	regmap_write(mc_map, MC_COM_CPU_RAW_INT_OFFSET(MC_CORE_ACPU),
		     (1 << MC_COM_INT_ACPU_DFS));
	return 0;
}

static int hisi_acpu_round_freq(unsigned int freq)
{
	unsigned int limit_flag, limit_freq = UINT_MAX;
	unsigned int max_freq;

	/* check the constrainted frequency */
	regmap_read(dfs_map, ACPU_DFS_FLAG, &limit_flag);
	if (limit_flag == ACPU_DFS_LOCK_FLAG)
		regmap_read(dfs_map, ACPU_DFS_FREQ_LMT, &limit_freq);

	/* check the supported maximum frequency */
	regmap_read(dfs_map, ACPU_DFS_FREQ_MAX, &max_freq);

	/* calculate the real maximum frequency */
	max_freq = min(max_freq, limit_freq);

	if (WARN_ON(freq > max_freq))
		freq = max_freq;

	return freq;
}

static unsigned long hisi_stub_clk_recalc_rate(struct clk_hw *hw,
		unsigned long parent_rate)
{
	u32 rate = 0;
	struct hisi_stub_clk *stub_clk = to_stub_clk(hw);
	unsigned long flags;

	BUG_ON(!stub_clk->lock);

	spin_lock_irqsave(stub_clk->lock, flags);

	switch (stub_clk->id) {
	case HISI_STUB_ACPU0:
		rate = hisi_acpu_get_freq();

		/* convert from KHz to Hz */
		rate *= 1000;
		break;

	default:
		pr_err("%s: un-supported clock id %d\n", __func__,
			stub_clk->id);
		break;
	}

	spin_unlock_irqrestore(stub_clk->lock, flags);
	return rate;
}

static int hisi_stub_clk_set_rate(struct clk_hw *hw, unsigned long rate,
                    unsigned long parent_rate)
{
	struct hisi_stub_clk *stub_clk = to_stub_clk(hw);
	unsigned long flags;
	unsigned long new_rate = rate / 1000;  /* Khz */
	int ret = 0;

	BUG_ON(!stub_clk->lock);

	spin_lock_irqsave(stub_clk->lock, flags);

	switch (stub_clk->id) {
	case HISI_STUB_ACPU0:
		ret = hisi_acpu_set_freq(new_rate);
		if (ret < 0) {
			spin_unlock_irqrestore(stub_clk->lock, flags);
			return ret;
		}

		break;

	default:
		pr_err("%s: un-supported clock id %d\n", __func__,
			stub_clk->id);
		break;
	}

	stub_clk->rate = new_rate;
	spin_unlock_irqrestore(stub_clk->lock, flags);

	pr_debug("%s: set rate=%ldKhz\n", __func__, new_rate);
	return ret;
}

static long hisi_stub_clk_round_rate(struct clk_hw *hw, unsigned long rate,
		unsigned long *parent_rate)
{
	struct hisi_stub_clk *stub_clk = to_stub_clk(hw);
	unsigned long flags;
	unsigned long new_rate = rate / 1000;  /* Khz */

	BUG_ON(!stub_clk->lock);

	spin_lock_irqsave(stub_clk->lock, flags);

	switch (stub_clk->id) {
	case HISI_STUB_ACPU0:
		new_rate = hisi_acpu_round_freq(new_rate);

		/* convert from KHz to Hz */
		new_rate *= 1000;
		break;

	default:
		pr_err("%s: un-supported clock id %d\n", __func__,
			stub_clk->id);
		break;
	}

	spin_unlock_irqrestore(stub_clk->lock, flags);
	return new_rate;
}

static struct clk_ops hisi_stub_clk_ops = {
	.recalc_rate	= hisi_stub_clk_recalc_rate,
	.round_rate	= hisi_stub_clk_round_rate,
	.set_rate	= hisi_stub_clk_set_rate,
};

static int hisi_stub_clk_init(struct device_node *np)
{
	int ret = 0;
	int max_freq;

	dfs_map = syscon_regmap_lookup_by_phandle(np,
				"hisilicon,clk-stub-sram");
	if (IS_ERR(dfs_map)) {
		ret = PTR_ERR(dfs_map);
		pr_err("failed to get sram regmap: %d\n", ret);
		return ret;
	}

	mc_map = syscon_regmap_lookup_by_phandle(np,
				"hisilicon,clk-stub-mc");
	if (IS_ERR(mc_map)) {
		ret = PTR_ERR(mc_map);
		pr_err("failed to get multi-core comm regmap: %d\n", ret);
		return ret;
	}

	/* initialize buffer to zero */
	regmap_write(dfs_map, ACPU_DFS_FLAG, 0x0);
	regmap_write(dfs_map, ACPU_DFS_FREQ_REQ, 0x0);
	regmap_write(dfs_map, ACPU_DFS_FREQ_LMT, 0x0);

	/* At boot time, set to maximum frequency */
	regmap_read(dfs_map, ACPU_DFS_FREQ_MAX, &max_freq);
	hisi_acpu_set_freq(max_freq);

	return ret;
}

static struct clk *_register_stub_clk(struct device *dev, unsigned int id,
	const char *name, const char *parent_name, unsigned long flags,
	spinlock_t *lock)
{
	struct hisi_stub_clk *stub_clk;
	struct clk *clk;
	struct clk_init_data init;

	stub_clk = kzalloc(sizeof(*stub_clk), GFP_KERNEL);
	if (!stub_clk) {
		pr_err("%s: fail to alloc stub clk!\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.ops = &hisi_stub_clk_ops;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;
	init.flags = flags;

	stub_clk->hw.init = &init;
	stub_clk->id = id;
	stub_clk->lock = lock;

	clk = clk_register(dev, &stub_clk->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: fail to register stub clk %s!\n", __func__, name);
		kfree(stub_clk);
	}

	return clk;
}

struct clk *hisi_register_stub_clk(struct device_node *np,
		unsigned int id, const char *name, const char *parent_name,
		unsigned long flags, spinlock_t *lock)
{
	int ret;
	struct clk *clk;

	pr_debug("[%s]: clk name = %s...\n", __func__, name);

	if (!initialized_stub_clk) {
		ret = hisi_stub_clk_init(np);
		if (ret)
			return ERR_PTR(-EINVAL);

		initialized_stub_clk = 1;
	}

	clk = _register_stub_clk(NULL, id, name, parent_name, flags, lock);
	return clk;
}
