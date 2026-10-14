/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * OMAP Voltage Management Routines
 *
 * Copyright (C) 2011, Texas Instruments, Inc.
 */

#ifndef __ARCH_ARM_OMAP_VOLTAGE_H
#define __ARCH_ARM_OMAP_VOLTAGE_H

/**
 * struct omap_volt_data - Omap voltage specific data.
 * @volt_nominal:	The possible voltage value in uV
 * @sr_efuse_offs:	The offset of the efuse register(from system
 *			control module base address) from where to read
 *			the n-target value for the smartreflex module.
 * @sr_errminlimit:	Error min limit value for smartreflex. This value
 *			differs at differnet opp and thus is linked
 *			with voltage.
 * @vp_errgain:		Error gain value for the voltage processor. This
 *			field also differs according to the voltage/opp.
 * @volt_calibrated:	The voltage in uV that SmartReflex class 1.5
 *			calibrated for this OPP, or 0 if not calibrated.
 *			Used instead of @volt_nominal when set.
 */
struct omap_volt_data {
	u32	volt_nominal;
	u32	sr_efuse_offs;
	u8	sr_errminlimit;
	u8	vp_errgain;
	u32	volt_calibrated;
};
struct voltagedomain;

struct voltagedomain *voltdm_lookup(const char *name);
unsigned long voltdm_get_voltage(struct voltagedomain *voltdm);
struct omap_volt_data *omap_voltage_get_voltdata(struct voltagedomain *voltdm,
		unsigned long volt);
#endif
