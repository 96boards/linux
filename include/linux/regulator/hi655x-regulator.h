/*
 * Copyright (c) 2015 Hisilicon Co. Ltd.
 *
 */

#ifndef __HISI_HI655X_REGULATOR_H__
#define __HISI_HI655X_REGULATOR_H__

enum hi655x_regulator_type {
	PMIC_BUCK_TYPE = 0,
	PMIC_LDO_TYPE  = 1,
	PMIC_LVS_TYPE  = 2,
	PMIC_BOOST_TYPE = 3,
	MTCMOS_SC_ON_TYPE  = 4,
	MTCMOS_ACPU_ON_TYPE = 5,
	SCHARGE_TYPE = 6,
};

struct hi655x_regulator_ctrl_regs {
	unsigned int enable_reg;
	unsigned int disable_reg;
	unsigned int status_reg;
};

struct hi655x_regulator_vset_regs {
	unsigned int vset_reg;
};

struct hi655x_regulator_ctrl_data {
	int shift;
	unsigned int mask;
};

struct hi655x_regulator_vset_data {
	int shift;
	unsigned int mask;
};

#endif
