// SPDX-License-Identifier: GPL-2.0-only
/*
 * pm.c - Common OMAP2+ power management-related code
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Copyright (C) 2010 Nokia Corporation
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/pm_opp.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/cpu.h>

#include <asm/system_misc.h>

#include "omap_device.h"
#include "common.h"

#include "soc.h"
#include "prcm-common.h"
#include "voltage.h"
#include "powerdomain.h"
#include "clockdomain.h"
#include "pm.h"

u32 enable_off_mode;

#ifdef CONFIG_SUSPEND
/*
 * omap_pm_suspend: points to a function that does the SoC-specific
 * suspend work
 */
static int (*omap_pm_suspend)(void);
#endif

#ifdef CONFIG_PM
/**
 * struct omap2_oscillator - Describe the board main oscillator latencies
 * @startup_time: oscillator startup latency
 * @shutdown_time: oscillator shutdown latency
 */
struct omap2_oscillator {
	u32 startup_time;
	u32 shutdown_time;
};

static struct omap2_oscillator oscillator = {
	.startup_time = ULONG_MAX,
	.shutdown_time = ULONG_MAX,
};

void omap_pm_get_oscillator(u32 *tstart, u32 *tshut)
{
	if (!tstart || !tshut)
		return;

	*tstart = oscillator.startup_time;
	*tshut = oscillator.shutdown_time;
}
#endif

int omap_pm_clkdms_setup(struct clockdomain *clkdm, void *unused)
{
	clkdm_allow_idle(clkdm);
	return 0;
}

#ifdef CONFIG_SUSPEND
static int omap_pm_enter(suspend_state_t suspend_state)
{
	int ret = 0;

	if (!omap_pm_suspend)
		return -ENOENT; /* XXX doublecheck */

	switch (suspend_state) {
	case PM_SUSPEND_MEM:
		ret = omap_pm_suspend();
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int omap_pm_begin(suspend_state_t state)
{
	cpu_idle_poll_ctrl(true);
	if (soc_is_omap34xx())
		omap_prcm_irq_prepare();
	return 0;
}

static void omap_pm_end(void)
{
	cpu_idle_poll_ctrl(false);
}

static void omap_pm_wake(void)
{
	if (soc_is_omap34xx())
		omap_prcm_irq_complete();
}

static const struct platform_suspend_ops omap_pm_ops = {
	.begin		= omap_pm_begin,
	.end		= omap_pm_end,
	.enter		= omap_pm_enter,
	.wake		= omap_pm_wake,
	.valid		= suspend_valid_only_mem,
};

/**
 * omap_common_suspend_init - Set common suspend routines for OMAP SoCs
 * @pm_suspend: function pointer to SoC specific suspend function
 */
void omap_common_suspend_init(void *pm_suspend)
{
	omap_pm_suspend = pm_suspend;
	suspend_set_ops(&omap_pm_ops);
}
#endif /* CONFIG_SUSPEND */

int __maybe_unused omap_pm_nop_init(void)
{
	return 0;
}

int (*omap_pm_soc_init)(void);

static int __init omap2_common_pm_late_init(void)
{
	if (!omap_pm_soc_init)
		return 0;

	/*
	 * Smartreflex device init. Stays at late_initcall: the smartreflex
	 * driver probes with this platform data.
	 */
	omap_devinit_smartreflex();

	return 0;
}
omap_late_initcall(omap2_common_pm_late_init);

/*
 * The voltage layer converts voltages to PMIC vsel values, and on TWL6030/
 * TWL6032 that needs SMPS_OFFSET read from the PMIC over I2C. The TWL can
 * come up as late as the deferred probe flush at late_initcall, so init the
 * voltage layer, and PM after it, at late_initcall_sync.
 */
static int __init omap2_common_pm_late_init_sync(void)
{
	int error;

	if (!omap_pm_soc_init)
		return 0;

	/* Init the voltage layer */
	omap3_twl_init();
	error = omap4_twl_init();
	if (error)
		error = omap4_cpcap_init();
	omap_voltage_late_init();

	/*
	 * Without PMIC data the VC is not programmed. If the PRCM then took
	 * a domain to retention, the VC would get no I2C ACK and trigger a
	 * global warm reset (OMAP4470 TRM section 3.8.5).
	 */
	if (cpu_is_omap44xx() && error) {
		pr_err("%s: no PMIC for the voltage layer, skipping PM init\n",
		       __func__);
		error = 0;
	} else {
		error = omap_pm_soc_init();

		/* Voltage domain regulators, e.g. the cpu0-supply for cpufreq */
		if (cpu_is_omap44xx())
			omap_vdd_regulator_init();
	}
	if (error)
		pr_warn("%s: pm soc init failed: %i\n", __func__, error);

	omap2_clk_enable_autoidle_all();

	return 0;
}
omap_late_initcall_sync(omap2_common_pm_late_init_sync);
