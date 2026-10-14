// SPDX-License-Identifier: GPL-2.0-only
/*
 * SmartReflex class 1.5 (software-triggered calibration)
 *
 * For each OPP, the voltage domain starts at the nominal voltage, the
 * SmartReflex and voltage processor loop runs until the voltage stops
 * moving, and then the loop is stopped. The voltage it settled at becomes
 * the OPP's voltage, the "calibrated" voltage, until the next calibration.
 * The loop stays off the rest of the time.
 *
 * OMAP4470 supports only this class [SWPU316 section 3.1]. It has the
 * temperature margins fused into the n-target values, so no margin is
 * added in software. AVS is mandatory at every OPP [SWPU325 Tables 2-1,
 * 2-3 and 2-5]. Calibration is repeated periodically for aging [OMAP4470
 * TRM section 3.1.1.3.3.1].
 *
 * Based on the class 1.5 driver in TI's 3.0 Android kernel by Nishanth
 * Menon <nm@ti.com>, Copyright (C) 2010-2011 Texas Instruments, Inc.
 * This version polls the VP voltage instead of using SR interrupts.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_qos.h>
#include <linux/power/smartreflex.h>
#include <linux/workqueue.h>

#include "soc.h"
#include "voltage.h"

/* Polling period and how long the voltage must not move to be settled */
#define SR1P5_POLL_US		1000
#define SR1P5_STABLE_POLLS	20
/* Give up after this many polls and take the highest recent voltage */
#define SR1P5_MAX_POLLS		1000

static unsigned int recal_hours = 24;
module_param(recal_hours, uint, 0444);
MODULE_PARM_DESC(recal_hours, "Recalibrate every N hours (0: never)");

/**
 * struct sr1p5 - class 1.5 state of one voltage domain
 * @sr:		the domain's SmartReflex module
 * @work:	calibration of the current OPP
 * @qos:	keeps the CPUs out of deep idle during calibration
 * @active:	autocompensation is on for this domain
 */
struct sr1p5 {
	struct omap_sr *sr;
	struct delayed_work work;
	struct pm_qos_request qos;
	bool active;
};

static struct sr1p5 sr1p5_doms[OMAP_SR_NR];
static struct delayed_work sr1p5_recal;

static unsigned long sr1p5_recal_delay(void)
{
	return (unsigned long)recal_hours * 3600 * HZ;
}

static struct sr1p5 *sr1p5_get(struct omap_sr *sr)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sr1p5_doms); i++)
		if (sr1p5_doms[i].sr == sr)
			return &sr1p5_doms[i];

	return NULL;
}

static u8 sr1p5_vp_vsel(struct voltagedomain *voltdm)
{
	struct omap_vp_instance *vp = voltdm->vp;

	return (voltdm->read(vp->voltage) & vp->common->vpvoltage_mask) >>
		__ffs(vp->common->vpvoltage_mask);
}

/*
 * SmartReflex convergence check [OMAP4470 TRM section 3.8.3.2.4]: the
 * error between the sensor counts and the n-target, in 0.1 % units, the
 * larger of the two sensors. Converged if it is within the error limits.
 */
static int sr1p5_error(struct omap_sr *sr, int *min, int *max)
{
	u32 senval = readl(sr->base + SENVAL);
	u32 nvalue = readl(sr->base + NVALUERECIPROCAL);
	u32 errconfig = readl(sr->base + ERRCONFIG_V2);
	int count[2] = { senval >> 16, senval & 0xffff };
	u32 gain[2] = { (nvalue >> NVALUERECIPROCAL_SENPGAIN_SHIFT) & 0xf,
			(nvalue >> NVALUERECIPROCAL_SENNGAIN_SHIFT) & 0xf };
	u32 rn[2] = { (nvalue >> NVALUERECIPROCAL_RNSENP_SHIFT) & 0xff,
		      (nvalue >> NVALUERECIPROCAL_RNSENN_SHIFT) & 0xff };
	int i, target, err = INT_MIN;

	for (i = 0; i < 2; i++) {
		if (!rn[i])
			continue;
		target = (1 << (gain[i] + 8)) / rn[i];
		err = max(err, (target - count[i]) * 1000 / target);
	}

	*min = (s8)(errconfig & SR_ERRMINLIMIT_MASK);
	*max = (s8)((errconfig & SR_ERRMAXLIMIT_MASK) >>
		    ERRCONFIG_ERRMAXLIMIT_SHIFT);

	return err;
}

/* Calibrate the current OPP of a domain. Called with voltdm->lock held. */
static void sr1p5_calibrate(struct sr1p5 *d)
{
	struct omap_sr *sr = d->sr;
	struct voltagedomain *voltdm = sr->voltdm;
	unsigned long nominal = voltdm->nominal_volt;
	struct omap_volt_data *vdata;
	u8 seen[SR1P5_STABLE_POLLS];
	u8 vsel, last, vmin, vmax;
	int i, stable = 0, err = 0, emin = 0, emax = 0;
	unsigned long uv;
	bool settled;
	ktime_t t0;
	int ret;

	vdata = omap_voltage_get_voltdata(voltdm, nominal);
	if (IS_ERR(vdata) || vdata->volt_calibrated)
		return;

	cpu_latency_qos_add_request(&d->qos, 0);

	/* Start at the nominal voltage, the safe pre-AVS voltage [SWPU325] */
	ret = voltdm->scale(voltdm, nominal);
	if (ret)
		goto out;

	omap_vp_enable(voltdm);
	ret = sr_enable(sr, nominal);
	if (ret) {
		omap_vp_disable(voltdm);
		goto out;
	}

	t0 = ktime_get();
	last = sr1p5_vp_vsel(voltdm);
	memset(seen, last, sizeof(seen));
	for (i = 0; i < SR1P5_MAX_POLLS && stable < SR1P5_STABLE_POLLS; i++) {
		usleep_range(SR1P5_POLL_US, SR1P5_POLL_US + 200);
		vsel = sr1p5_vp_vsel(voltdm);
		seen[i % SR1P5_STABLE_POLLS] = vsel;
		stable = vsel == last ? stable + 1 : 0;
		last = vsel;
	}
	settled = stable == SR1P5_STABLE_POLLS;
	if (settled)
		err = sr1p5_error(sr, &emin, &emax);

	sr_disable_errgen(sr);
	omap_vp_disable(voltdm);
	sr_disable(sr);

	/* Still moving: take the highest voltage of the last polls */
	vsel = sr1p5_vp_vsel(voltdm);
	if (!settled)
		for (i = 0; i < SR1P5_STABLE_POLLS; i++)
			vsel = max(vsel, seen[i]);

	vmin = voltdm->pmic->uv_to_vsel(max(voltdm->vp_param->vddmin,
					    voltdm->pmic->vddmin));
	vmax = voltdm->pmic->uv_to_vsel(min(voltdm->vp_param->vddmax,
					    voltdm->pmic->vddmax));
	if (vsel < vmin || vsel > vmax) {
		pr_err("vdd_%s: SR class 1.5: vsel 0x%02x outside the VP limits, keeping %lu uV\n",
		       voltdm->name, vsel, nominal);
		goto out_nominal;
	}

	uv = voltdm->pmic->vsel_to_uv(vsel);
	if (uv > nominal)
		pr_warn("vdd_%s: SR class 1.5: %lu uV is above the nominal %lu uV\n",
			voltdm->name, uv, nominal);
	if (vsel == vmin || vsel == vmax)
		pr_warn("vdd_%s: SR class 1.5: stopped at the VP limit\n",
			voltdm->name);

	/* Set first: the scale looks the OPP up by this voltage */
	vdata->volt_calibrated = uv;
	ret = voltdm->scale(voltdm, uv);
	if (ret) {
		vdata->volt_calibrated = 0;
		goto out_nominal;
	}

	if (settled)
		pr_info("vdd_%s: SR class 1.5: %lu -> %lu uV in %lld ms, error %s%d.%d%% (limits %d%%..%d%%)\n",
			voltdm->name, nominal, uv,
			ktime_ms_delta(ktime_get(), t0), err < 0 ? "-" : "",
			abs(err) / 10, abs(err) % 10, emin, emax);
	else
		pr_warn("vdd_%s: SR class 1.5: %lu -> %lu uV, not settled after %lld ms\n",
			voltdm->name, nominal, uv,
			ktime_ms_delta(ktime_get(), t0));
	goto out;

out_nominal:
	voltdm->scale(voltdm, nominal);
out:
	if (ret)
		pr_err("vdd_%s: SR class 1.5: calibration failed: %d\n",
		       voltdm->name, ret);
	cpu_latency_qos_remove_request(&d->qos);
}

static void sr1p5_work(struct work_struct *work)
{
	struct sr1p5 *d = container_of(to_delayed_work(work), struct sr1p5,
				       work);
	struct voltagedomain *voltdm = d->sr->voltdm;

	mutex_lock(&voltdm->lock);
	if (d->active)
		sr1p5_calibrate(d);
	mutex_unlock(&voltdm->lock);
}

/* Forget a domain's calibrations and go back to the nominal voltage */
static void sr1p5_reset(struct sr1p5 *d)
{
	struct voltagedomain *voltdm = d->sr->voltdm;
	int i;

	mutex_lock(&voltdm->lock);
	for (i = 0; voltdm->volt_data[i].volt_nominal; i++)
		voltdm->volt_data[i].volt_calibrated = 0;
	voltdm->scale(voltdm, voltdm->nominal_volt);
	mutex_unlock(&voltdm->lock);
}

static void sr1p5_recal_work(struct work_struct *work)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sr1p5_doms); i++) {
		struct sr1p5 *d = &sr1p5_doms[i];

		if (!d->sr || !d->active)
			continue;
		pr_info("vdd_%s: SR class 1.5: recalibrating\n",
			d->sr->voltdm->name);
		sr1p5_reset(d);
		queue_delayed_work(system_freezable_wq, &d->work, 0);
	}

	queue_delayed_work(system_freezable_power_efficient_wq,
			   &sr1p5_recal, sr1p5_recal_delay());
}

static int sr1p5_enable(struct omap_sr *sr)
{
	struct sr1p5 *d = sr1p5_get(sr);

	if (!d) {
		d = sr1p5_get(NULL);
		if (!d)
			return -ENOMEM;
		d->sr = sr;
		INIT_DELAYED_WORK(&d->work, sr1p5_work);
	}

	if (!sr->voltdm->vp || !sr->voltdm->scale || !sr->voltdm->nominal_volt)
		return -ENODEV;

	d->active = true;
	queue_delayed_work(system_freezable_wq, &d->work, 0);

	return 0;
}

static int sr1p5_disable(struct omap_sr *sr, int is_volt_reset)
{
	struct sr1p5 *d = sr1p5_get(sr);

	if (!d)
		return 0;

	d->active = false;
	cancel_delayed_work_sync(&d->work);
	if (is_volt_reset)
		sr1p5_reset(d);

	return 0;
}

static int sr1p5_configure(struct omap_sr *sr)
{
	return sr_configure_errgen(sr);
}

static struct omap_sr_class_data sr1p5_class = {
	.enable = sr1p5_enable,
	.disable = sr1p5_disable,
	.configure = sr1p5_configure,
	.class_type = SR_CLASS1P5,
};

/**
 * omap_sr_class1p5_scaled() - a voltage domain went to another OPP
 * @voltdm:	the voltage domain
 *
 * Called by the voltage layer after a scale, without voltdm->lock held.
 * Calibrates the new OPP if it has not been calibrated yet.
 */
void omap_sr_class1p5_scaled(struct voltagedomain *voltdm)
{
	struct omap_volt_data *vdata;
	int i;

	for (i = 0; i < ARRAY_SIZE(sr1p5_doms); i++) {
		struct sr1p5 *d = &sr1p5_doms[i];

		if (!d->sr || d->sr->voltdm != voltdm || !d->active)
			continue;
		vdata = omap_voltage_get_voltdata(voltdm, voltdm->nominal_volt);
		if (!IS_ERR(vdata) && !vdata->volt_calibrated)
			queue_delayed_work(system_freezable_wq, &d->work, 0);
	}
}

/**
 * omap_sr_class1p5_init() - register class 1.5 and calibrate every domain
 *
 * Called once the voltage layer and the PMIC are set up.
 */
int __init omap_sr_class1p5_init(void)
{
	static const char * const names[] = { "mpu", "iva", "core" };
	int i, ret;

	ret = sr_register_class(&sr1p5_class);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		ret = omap_sr_autocomp_start(voltdm_lookup(names[i]));
		if (ret)
			pr_warn("vdd_%s: SR class 1.5 not started: %d\n",
				names[i], ret);
	}

	INIT_DELAYED_WORK(&sr1p5_recal, sr1p5_recal_work);
	if (recal_hours)
		queue_delayed_work(system_freezable_power_efficient_wq,
				   &sr1p5_recal, sr1p5_recal_delay());

	pr_info("SmartReflex class 1.5 initialized, recalibration every %u h\n",
		recal_hours);

	return 0;
}
