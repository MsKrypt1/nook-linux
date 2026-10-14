// SPDX-License-Identifier: GPL-2.0-only
/*
 * omap4-restart.c - Common to OMAP4 and OMAP5
 */

#include <linux/types.h>
#include <linux/reboot.h>
#include "common.h"
#include "prm.h"
#include "prminst44xx.h"
#include "soc.h"
#include "voltage.h"

/**
 * omap44xx_restart - trigger a software restart of the SoC
 * @mode: the "reboot mode", see arch/arm/kernel/{setup,process}.c
 * @cmd: passed from the userspace program rebooting the system (if provided)
 *
 * Resets the SoC.  For @cmd, see the 'reboot' syscall in
 * kernel/sys.c.  No return value.
 */
void omap44xx_restart(enum reboot_mode mode, const char *cmd)
{
	/* XXX Should save 'cmd' into scratchpad for use after reboot */

	/*
	 * A warm reset keeps the DRAM contents, so pstore/ramoops survives
	 * it; after a cold reset they are gone. Warm is safe on OMAP4470
	 * now that PRM_RSTST is cleared at boot (B&N's X-Loader hangs on a
	 * stale cold-reset bit), but not with a SmartReflex/VP loop running
	 * [errata i724].
	 */
	if (soc_is_omap447x() && omap_sr_class1p5_busy()) {
		omap4_prminst_global_cold_sw_reset();
		return;
	}

	omap_prm_reset_system();
}
