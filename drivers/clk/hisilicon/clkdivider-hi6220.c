/*
 * Hisilicon hi6220 SoC divider clock driver
 *
 * Copyright (c) 2015 Hisilicon Limited.
 *
 * Author: Bintian Wang <bintian.wang@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/kernel.h>
#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/err.h>

#define div_mask(width)	((1 << (width)) - 1)

/*
 * The reverse of DIV_ROUND_UP: The maximum number which
 * divided by m is r
 */
#define MULT_ROUND_UP(r, m) ((r) * (m) + (m) - 1)

/**
 * struct hi6220_clk_divider - divider clock for hi6220
 *
 * @hw:		handle between common and hardware-specific interfaces
 * @reg:	register containing divider
 * @shift:	shift to the divider bit field
 * @width:	width of the divider bit field
 * @mask:	mask for setting divider rate
 * @table:	the div table that the divider supports
 * @lock:	register lock
 */
struct hi6220_clk_divider {
	struct clk_hw	hw;
	void __iomem	*reg;
	u8		shift;
	u8		width;
	u32		mask;
	const struct clk_div_table *table;
	spinlock_t	*lock;
};

#define to_hi6220_clk_divider(_hw)	\
	container_of(_hw, struct hi6220_clk_divider, hw)

static unsigned int hi6220_get_table_maxdiv(const struct clk_div_table *table)
{
	unsigned int maxdiv = 0;
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->div > maxdiv)
			maxdiv = clkt->div;
	return maxdiv;
}

static unsigned int hi6220_get_table_div(const struct clk_div_table *table,
					unsigned int val)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->val == val)
			return clkt->div;

	return 0;
}

static unsigned int hi6220_get_table_val(const struct clk_div_table *table,
					unsigned int div)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->div == div)
			return clkt->val;
	return 0;
}

static unsigned long hi6220_clkdiv_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	unsigned int div, val;
	struct hi6220_clk_divider *dclk = to_hi6220_clk_divider(hw);

	val = readl_relaxed(dclk->reg) >> dclk->shift;
	val &= div_mask(dclk->width);

	div = hi6220_get_table_div(dclk->table, val);
	if (!div) {
		pr_warn("%s: Invalid divisor for clock %s\n", __func__,
			   __clk_get_name(hw->clk));
		return parent_rate;
	}

	return parent_rate / div;
}

static bool hi6220_is_valid_div(const struct clk_div_table *table,
				unsigned int div)
{
	const struct clk_div_table *clkt;

	for (clkt = table; clkt->div; clkt++)
		if (clkt->div == div)
			return true;
	return false;
}

static int hi6220_clkdiv_bestdiv(struct clk_hw *hw, unsigned long rate,
				 unsigned long *best_parent_rate)
{
	unsigned int i, bestdiv = 0;
	unsigned long parent_rate, best = 0, now, maxdiv;

	struct hi6220_clk_divider *dclk = to_hi6220_clk_divider(hw);
	struct clk *clk_parent = __clk_get_parent(hw->clk);

	maxdiv = hi6220_get_table_maxdiv(dclk->table);

	if (!(__clk_get_flags(hw->clk) & CLK_SET_RATE_PARENT)) {
		parent_rate = *best_parent_rate;
		bestdiv = DIV_ROUND_UP(parent_rate, rate);
		bestdiv = (bestdiv == 0) ? 1 : bestdiv;
		bestdiv = (bestdiv > maxdiv) ? maxdiv : bestdiv;
		return bestdiv;
	}

	/*
	 * The maximum divider we can use without overflowing
	 * unsigned long in rate * i below
	 */
	maxdiv = min(ULONG_MAX / rate, maxdiv);

	for (i = 1; i <= maxdiv; i++) {
		if (!hi6220_is_valid_div(dclk->table, i))
			continue;
		parent_rate = __clk_round_rate(clk_parent,
					MULT_ROUND_UP(rate, i));
		now = parent_rate / i;
		if (now <= rate && now > best) {
			bestdiv = i;
			best = now;
			*best_parent_rate = parent_rate;
		}
	}

	if (!bestdiv) {
		bestdiv = hi6220_get_table_maxdiv(dclk->table);
		*best_parent_rate = __clk_round_rate(clk_parent, 1);
	}

	return bestdiv;
}

static long hi6220_clkdiv_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	int div;

	if (!rate)
		rate = 1;

	div = hi6220_clkdiv_bestdiv(hw, rate, prate);

	return *prate / div;
}

static int hi6220_clkdiv_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	unsigned int div, value;
	unsigned long flags = 0;
	u32 data;
	struct hi6220_clk_divider *dclk = to_hi6220_clk_divider(hw);

	div = parent_rate / rate;

	if (!hi6220_is_valid_div(dclk->table, div))
		return -EINVAL;

	value = hi6220_get_table_val(dclk->table, div);

	if (value > div_mask(dclk->width))
		value = div_mask(dclk->width);

	if (dclk->lock)
		spin_lock_irqsave(dclk->lock, flags);

	data = readl_relaxed(dclk->reg);
	data &= ~(div_mask(dclk->width) << dclk->shift);
	data |= value << dclk->shift;
	data |= dclk->mask;

	writel_relaxed(data, dclk->reg);

	if (dclk->lock)
		spin_unlock_irqrestore(dclk->lock, flags);

	return 0;
}

static struct clk_ops hi6220_clkdiv_ops = {
	.recalc_rate = hi6220_clkdiv_recalc_rate,
	.round_rate = hi6220_clkdiv_round_rate,
	.set_rate = hi6220_clkdiv_set_rate,
};

struct clk *hi6220_register_clkdiv(struct device *dev, const char *name,
	const char *parent_name, unsigned long flags, void __iomem *reg,
	u8 shift, u8 width, u32 mask_bit, spinlock_t *lock)
{
	struct hi6220_clk_divider *div;
	struct clk *clk;
	struct clk_init_data init;
	struct clk_div_table *table;
	u32 max_div, min_div;
	int i;

	/* allocate the divider */
	div = kzalloc(sizeof(struct hi6220_clk_divider), GFP_KERNEL);
	if (!div) {
		pr_err("%s: could not allocate divider clk\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* Init the divider table */
	max_div = div_mask(width) + 1;
	min_div = 1;

	table = kzalloc(sizeof(struct clk_div_table) * (max_div + 1), GFP_KERNEL);
	if (!table) {
		kfree(div);
		pr_err("%s: failed to alloc divider table!\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < max_div; i++) {
		table[i].div = min_div + i;
		table[i].val = table[i].div - 1;
	}

	init.name = name;
	init.ops = &hi6220_clkdiv_ops;
	init.flags = flags | CLK_IS_BASIC;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;

	/* struct hi6220_clk_divider assignments */
	div->reg = reg;
	div->shift = shift;
	div->width = width;
	div->mask = mask_bit ? BIT(mask_bit) : 0;
	div->lock = lock;
	div->hw.init = &init;
	div->table = table;

	/* register the clock */
	clk = clk_register(dev, &div->hw);

	if (IS_ERR(clk)) {
		kfree(table);
		kfree(div);
	}

	return clk;
}
