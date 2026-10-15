// SPDX-License-Identifier: GPL-2.0-only
/*
 * OMAP4-specific DPLL control functions
 *
 * Copyright (C) 2011 Texas Instruments, Inc.
 * Rajendra Nayak
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk/ti.h>

#include "clock.h"

/*
 * Maximum DPLL input frequency (FINT) and output frequency (FOUT) that
 * can supported when using the DPLL low-power mode. Frequencies are
 * defined in OMAP4430/60 Public TRM section 3.6.3.3.2 "Enable Control,
 * Status, and Low-Power Operation Mode".
 */
#define OMAP4_DPLL_LP_FINT_MAX	1000000
#define OMAP4_DPLL_LP_FOUT_MAX	100000000

/*
 * Bitfield declarations
 */
#define OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK		BIT(8)
#define OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK		BIT(10)
#define OMAP4430_DPLL_REGM4XEN_MASK			BIT(11)

/* Static rate multiplier for OMAP4 REGM4XEN clocks */
#define OMAP4430_REGM4XEN_MULT				4

static void omap4_dpllmx_allow_gatectrl(struct clk_hw_omap *clk)
{
	u32 v;
	u32 mask;

	if (!clk)
		return;

	mask = clk->flags & CLOCK_CLKOUTX2 ?
			OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK :
			OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK;

	v = ti_clk_ll_ops->clk_readl(&clk->clksel_reg);
	/* Clear the bit to allow gatectrl */
	v &= ~mask;
	ti_clk_ll_ops->clk_writel(v, &clk->clksel_reg);
}

static void omap4_dpllmx_deny_gatectrl(struct clk_hw_omap *clk)
{
	u32 v;
	u32 mask;

	if (!clk)
		return;

	mask = clk->flags & CLOCK_CLKOUTX2 ?
			OMAP4430_DPLL_CLKOUTX2_GATE_CTRL_MASK :
			OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK;

	v = ti_clk_ll_ops->clk_readl(&clk->clksel_reg);
	/* Set the bit to deny gatectrl */
	v |= mask;
	ti_clk_ll_ops->clk_writel(v, &clk->clksel_reg);
}

const struct clk_hw_omap_ops clkhwops_omap4_dpllmx = {
	.allow_idle	= omap4_dpllmx_allow_gatectrl,
	.deny_idle      = omap4_dpllmx_deny_gatectrl,
};

/**
 * omap4_dpll_lpmode_recalc - compute DPLL low-power setting
 * @dd: pointer to the dpll data structure
 *
 * Calculates if low-power mode can be enabled based upon the last
 * multiplier and divider values calculated. If low-power mode can be
 * enabled, then the bit to enable low-power mode is stored in the
 * last_rounded_lpmode variable. This implementation is based upon the
 * criteria for enabling low-power mode as described in the OMAP4430/60
 * Public TRM section 3.6.3.3.2 "Enable Control, Status, and Low-Power
 * Operation Mode".
 */
static void omap4_dpll_lpmode_recalc(struct dpll_data *dd)
{
	long fint, fout;

	fint = clk_hw_get_rate(dd->clk_ref) / (dd->last_rounded_n + 1);
	fout = fint * dd->last_rounded_m;

	if ((fint < OMAP4_DPLL_LP_FINT_MAX) && (fout < OMAP4_DPLL_LP_FOUT_MAX))
		dd->last_rounded_lpmode = 1;
	else
		dd->last_rounded_lpmode = 0;
}

/**
 * omap4_dpll_regm4xen_recalc - compute DPLL rate, considering REGM4XEN bit
 * @hw: pointer to the clock to compute the rate for
 * @parent_rate: clock rate of the DPLL parent
 *
 * Compute the output rate for the OMAP4 DPLL represented by @clk.
 * Takes the REGM4XEN bit into consideration, which is needed for the
 * OMAP4 ABE DPLL.  Returns the DPLL's output rate (before M-dividers)
 * upon success, or 0 upon error.
 */
unsigned long omap4_dpll_regm4xen_recalc(struct clk_hw *hw,
					 unsigned long parent_rate)
{
	struct clk_hw_omap *clk = to_clk_hw_omap(hw);
	u32 v;
	unsigned long rate;
	struct dpll_data *dd;

	if (!clk || !clk->dpll_data)
		return 0;

	dd = clk->dpll_data;

	rate = omap2_get_dpll_rate(clk);

	/* regm4xen adds a multiplier of 4 to DPLL calculations */
	v = ti_clk_ll_ops->clk_readl(&dd->control_reg);
	if (v & OMAP4430_DPLL_REGM4XEN_MASK)
		rate *= OMAP4430_REGM4XEN_MULT;

	return rate;
}

/**
 * omap4_dpll_regm4xen_round_rate - round DPLL rate, considering REGM4XEN bit
 * @hw: struct hw_clk containing the struct clk * of the DPLL to round a rate for
 * @target_rate: the desired rate of the DPLL
 * @parent_rate: clock rate of the DPLL parent
 *
 * Compute the rate that would be programmed into the DPLL hardware
 * for @clk if set_rate() were to be provided with the rate
 * @target_rate.  Takes the REGM4XEN bit into consideration, which is
 * needed for the OMAP4 ABE DPLL.  Returns the rounded rate (before
 * M-dividers) upon success, -EINVAL if @clk is null or not a DPLL, or
 * ~0 if an error occurred in omap2_dpll_round_rate().
 */
long omap4_dpll_regm4xen_round_rate(struct clk_hw *hw,
				    unsigned long target_rate,
				    unsigned long *parent_rate)
{
	struct clk_hw_omap *clk = to_clk_hw_omap(hw);
	struct dpll_data *dd;
	long r;

	if (!clk || !clk->dpll_data)
		return -EINVAL;

	dd = clk->dpll_data;

	dd->last_rounded_m4xen = 0;

	/*
	 * First try to compute the DPLL configuration for
	 * target rate without using the 4X multiplier.
	 */
	r = omap2_dpll_round_rate(hw, target_rate, NULL);
	if (r != ~0)
		goto out;

	/*
	 * If we did not find a valid DPLL configuration, try again, but
	 * this time see if using the 4X multiplier can help. Enabling the
	 * 4X multiplier is equivalent to dividing the target rate by 4.
	 */
	r = omap2_dpll_round_rate(hw, target_rate / OMAP4430_REGM4XEN_MULT,
				  NULL);
	if (r == ~0)
		return r;

	dd->last_rounded_rate *= OMAP4430_REGM4XEN_MULT;
	dd->last_rounded_m4xen = 1;

out:
	omap4_dpll_lpmode_recalc(dd);

	return dd->last_rounded_rate;
}

/**
 * omap4_dpll_regm4xen_determine_rate - determine rate for a DPLL
 * @hw: pointer to the clock to determine rate for
 * @req: target rate request
 *
 * Determines which DPLL mode to use for reaching a desired rate.
 * Checks whether the DPLL shall be in bypass or locked mode, and if
 * locked, calculates the M,N values for the DPLL via round-rate.
 * Returns 0 on success and a negative error value otherwise.
 */
int omap4_dpll_regm4xen_determine_rate(struct clk_hw *hw,
				       struct clk_rate_request *req)
{
	struct clk_hw_omap *clk = to_clk_hw_omap(hw);
	struct dpll_data *dd;

	if (!req->rate)
		return -EINVAL;

	dd = clk->dpll_data;
	if (!dd)
		return -EINVAL;

	if (clk_hw_get_rate(dd->clk_bypass) == req->rate &&
	    (dd->modes & (1 << DPLL_LOW_POWER_BYPASS))) {
		req->best_parent_hw = dd->clk_bypass;
	} else {
		req->rate = omap4_dpll_regm4xen_round_rate(hw, req->rate,
						&req->best_parent_rate);
		req->best_parent_hw = dd->clk_ref;
	}

	req->best_parent_rate = req->rate;

	return 0;
}

/*
 * OMAP4470 DPLL_MPU [OMAP4470 TRM §3.6.3.7]: below 1 GHz MPU_DPLL_CLK is
 * CLKOUT_M2 = Fdpll / (2 * M2), with the DPLL locked at 1.6 GHz for both
 * OPP50 (M2 = 2) and OPP100 (M2 = 1). At or above 1 GHz it is CLKOUTX2_M3
 * (M3 hard-wired to 1) through the duty-cycle corrector, so the DPLL locks
 * at the MPU rate itself and DCC_EN is set [TRM p422, Table 3-88].
 * Fdpll = 2 * M / (N + 1) * ref. dpll_mpu_ck stands for MPU_DPLL_CLK, so
 * its rate covers M2 and DCC_EN.
 *
 * A locked DPLL with DCC_EN = 1 must not go straight to bypass: DCC_EN is
 * cleared while locked first [TRM p422 CAUTION]. So DCC is only switched
 * while the DPLL is locked, and every relock runs with DCC_EN = 0.
 */
#define OMAP4470_MPU_DCC_RATE		1000000000
#define OMAP4470_MPU_M2_MAX_RATE	800000000	/* Fdpll <= 1.6 GHz */
#define OMAP4470_MPU_BRIDGE_DIV_RATE	800000000	/* SWPU325 p12 */

#define OMAP4470_DPLL_DCC_EN_MASK	BIT(22)
#define OMAP4470_DPLL_DCC_COUNT_MAX_MASK	GENMASK(31, 24)
#define OMAP4470_DPLL_CLKOUT_DIV_MASK	GENMASK(4, 0)
#define OMAP4470_DPLL_CLKOUT_DIVCHACK_MASK	BIT(5)
#define OMAP4470_MPU_EMIF_DIV_MODE_MASK	BIT(24)
#define OMAP4470_MPU_ABE_DIV_MODE_MASK	BIT(25)

/*
 * DCC ramp step: 32 * DCC_COUNT_MAX L4 (100 MHz) cycles, at least 2.5 us
 * for the DCC to lock [TRM p422, p825]. The reset value 5 gives 1.6 us;
 * 16 gives 5.1 us, which leaves margin should L4 run faster.
 */
#define OMAP4470_DPLL_DCC_COUNT_MAX	16

#define OMAP4470_M2_WAIT_TRIES		100

struct omap4470_mpu_cfg {
	unsigned long lock_rate;	/* M / (N + 1) * ref = Fdpll / 2 */
	unsigned long rate;
	u32 m2;
	bool dcc;
};

static void omap4470_mpu_dpll_rmw(struct clk_omap_reg *reg, u32 mask, u32 val)
{
	u32 v;

	v = ti_clk_ll_ops->clk_readl(reg);
	v &= ~mask;
	v |= val & mask;
	ti_clk_ll_ops->clk_writel(v, reg);
}

/*
 * Pick the DPLL setting for @rate and round it. Leaves the M/N for the
 * lock rate in dd->last_rounded_*, as omap2_dpll_round_rate() does.
 */
static int omap4470_mpu_dpll_round(struct clk_hw *hw, unsigned long rate,
				   struct omap4470_mpu_cfg *cfg)
{
	long r;

	if (rate >= OMAP4470_MPU_DCC_RATE) {
		cfg->dcc = true;
		cfg->m2 = 1;
		r = omap2_dpll_round_rate(hw, rate / 2, NULL);
		if (r == ~0)
			return -EINVAL;
		cfg->lock_rate = r;
		cfg->rate = r * 2;
		return 0;
	}

	/* Nothing between 800 MHz and 1 GHz: round down */
	rate = min_t(unsigned long, rate, OMAP4470_MPU_M2_MAX_RATE);
	cfg->dcc = false;
	cfg->m2 = clamp_t(u32, OMAP4470_MPU_M2_MAX_RATE / rate, 1,
			  FIELD_MAX(OMAP4470_DPLL_CLKOUT_DIV_MASK));
	r = omap2_dpll_round_rate(hw, rate * cfg->m2, NULL);
	if (r == ~0)
		return -EINVAL;
	cfg->lock_rate = r;
	cfg->rate = r / cfg->m2;

	return 0;
}

static u32 omap4470_mpu_dpll_get_m2(struct dpll_data *dd)
{
	u32 m2;

	m2 = ti_clk_ll_ops->clk_readl(&dd->m2_div_reg);
	m2 = FIELD_GET(OMAP4470_DPLL_CLKOUT_DIV_MASK, m2);

	return m2 ? m2 : 1;
}

static int omap4470_mpu_dpll_set_m2(struct clk_hw *hw, u32 m2)
{
	struct dpll_data *dd = to_clk_hw_omap(hw)->dpll_data;
	u32 ack, v;
	int i;

	v = ti_clk_ll_ops->clk_readl(&dd->m2_div_reg);
	ack = v & OMAP4470_DPLL_CLKOUT_DIVCHACK_MASK;
	v &= ~OMAP4470_DPLL_CLKOUT_DIV_MASK;
	v |= FIELD_PREP(OMAP4470_DPLL_CLKOUT_DIV_MASK, m2);
	ti_clk_ll_ops->clk_writel(v, &dd->m2_div_reg);

	/* DIVCHACK toggles once the new divider is in effect [TRM p827] */
	for (i = 0; i < OMAP4470_M2_WAIT_TRIES; i++) {
		v = ti_clk_ll_ops->clk_readl(&dd->m2_div_reg);
		if ((v & OMAP4470_DPLL_CLKOUT_DIVCHACK_MASK) != ack)
			return 0;
		udelay(1);
	}

	pr_err("clock: %s: M2 = %u not acknowledged\n", clk_hw_get_name(hw),
	       m2);
	return -ETIMEDOUT;
}

static void omap4470_mpu_dpll_set_bridge_div(struct dpll_data *dd, bool high)
{
	u32 mask = OMAP4470_MPU_EMIF_DIV_MODE_MASK |
		   OMAP4470_MPU_ABE_DIV_MODE_MASK;

	omap4470_mpu_dpll_rmw(&dd->mpu_clkctrl_reg, mask, high ? mask : 0);
}

/*
 * Besides the DPLL autoidle, let CLKOUT_M2 gate when nothing needs it
 * (DPLL_CLKOUT_GATE_CTRL = 0). The bootloader leaves it forced on.
 */
static void omap4470_mpu_dpll_allow_idle(struct clk_hw_omap *clk)
{
	clkhwops_omap3_dpll.allow_idle(clk);
	omap4470_mpu_dpll_rmw(&clk->dpll_data->m2_div_reg,
			      OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK, 0);
}

static void omap4470_mpu_dpll_deny_idle(struct clk_hw_omap *clk)
{
	clkhwops_omap3_dpll.deny_idle(clk);
	omap4470_mpu_dpll_rmw(&clk->dpll_data->m2_div_reg,
			      OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK,
			      OMAP4430_DPLL_CLKOUT_GATE_CTRL_MASK);
}

const struct clk_hw_omap_ops clkhwops_omap4470_mpu_dpll = {
	.allow_idle	= omap4470_mpu_dpll_allow_idle,
	.deny_idle	= omap4470_mpu_dpll_deny_idle,
};

unsigned long omap4470_mpu_dpll_recalc(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct clk_hw_omap *clk = to_clk_hw_omap(hw);
	struct dpll_data *dd = clk->dpll_data;
	unsigned long rate;

	rate = omap2_get_dpll_rate(clk);

	if (ti_clk_ll_ops->clk_readl(&dd->mult_div1_reg) &
	    OMAP4470_DPLL_DCC_EN_MASK)
		return rate * 2;

	return rate / omap4470_mpu_dpll_get_m2(dd);
}

int omap4470_mpu_dpll_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	struct dpll_data *dd = to_clk_hw_omap(hw)->dpll_data;
	struct omap4470_mpu_cfg cfg;
	int ret;

	if (!req->rate)
		return -EINVAL;

	ret = omap4470_mpu_dpll_round(hw, req->rate, &cfg);
	if (ret)
		return ret;

	req->rate = cfg.rate;
	req->best_parent_hw = dd->clk_ref;
	req->best_parent_rate = clk_hw_get_rate(dd->clk_ref);

	return 0;
}

int omap4470_mpu_dpll_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct clk_hw_omap *clk = to_clk_hw_omap(hw);
	struct dpll_data *dd = clk->dpll_data;
	unsigned long old_rate = clk_hw_get_rate(hw);
	struct omap4470_mpu_cfg cfg;
	u32 v, m, n, m2;
	int ret;

	if (clk_hw_get_parent(hw) != dd->clk_ref)
		return -EINVAL;

	ret = omap4470_mpu_dpll_round(hw, rate, &cfg);
	if (ret)
		return ret;

	v = ti_clk_ll_ops->clk_readl(&dd->mult_div1_reg);
	m = (v & dd->mult_mask) >> __ffs(dd->mult_mask);
	n = ((v & dd->div1_mask) >> __ffs(dd->div1_mask)) + 1;
	m2 = omap4470_mpu_dpll_get_m2(dd);

	pr_debug("clock: %s: %lu -> %lu Hz: M/N %u/%u -> %u/%u, M2 %u -> %u, DCC %d -> %d\n",
		 clk_hw_get_name(hw), old_rate, cfg.rate, m, n - 1,
		 dd->last_rounded_m, dd->last_rounded_n - 1, m2, cfg.m2,
		 !!(v & OMAP4470_DPLL_DCC_EN_MASK), cfg.dcc);

	/* Bridge dividers up before going above 800 MHz [SWPU325 p12] */
	if (cfg.rate > OMAP4470_MPU_BRIDGE_DIV_RATE)
		omap4470_mpu_dpll_set_bridge_div(dd, true);

	/* Back to CLKOUT_M2 (Fdpll / 2, M2 is 1 here) while still locked */
	if (v & OMAP4470_DPLL_DCC_EN_MASK)
		omap4470_mpu_dpll_rmw(&dd->mult_div1_reg,
				      OMAP4470_DPLL_DCC_EN_MASK, 0);

	/*
	 * Order M2 and the relock so that no step runs the MPU faster than
	 * both the old and the new rate.
	 */
	if (cfg.m2 > m2) {
		ret = omap4470_mpu_dpll_set_m2(hw, cfg.m2);
		if (ret)
			return ret;
	}

	if (m != dd->last_rounded_m || n != dd->last_rounded_n) {
		ret = omap3_noncore_dpll_set_rate(hw, cfg.lock_rate,
						  parent_rate);
		if (ret)
			return ret;
		if (!(ti_clk_ll_ops->clk_readl(&dd->idlest_reg) &
		      dd->idlest_mask))
			return -EIO;
	}

	if (cfg.m2 < m2) {
		ret = omap4470_mpu_dpll_set_m2(hw, cfg.m2);
		if (ret)
			return ret;
	}

	if (cfg.dcc) {
		omap4470_mpu_dpll_rmw(&dd->mult_div1_reg,
				      OMAP4470_DPLL_DCC_COUNT_MAX_MASK,
				      FIELD_PREP(OMAP4470_DPLL_DCC_COUNT_MAX_MASK,
						 OMAP4470_DPLL_DCC_COUNT_MAX));
		omap4470_mpu_dpll_rmw(&dd->mult_div1_reg,
				      OMAP4470_DPLL_DCC_EN_MASK,
				      OMAP4470_DPLL_DCC_EN_MASK);
	}

	/* Bridge dividers down only once at or below 800 MHz */
	if (cfg.rate <= OMAP4470_MPU_BRIDGE_DIV_RATE)
		omap4470_mpu_dpll_set_bridge_div(dd, false);

	return 0;
}
