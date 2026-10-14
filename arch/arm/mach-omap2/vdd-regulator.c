// SPDX-License-Identifier: GPL-2.0-only
/*
 * OMAP voltage domain regulator
 *
 * Exposes the VC/VP voltage control of an OMAP voltage domain as a
 * regulator, so that cpufreq-dt can scale VDD_MPU through "cpu0-supply".
 * On the TWL6030/TWL6032 the SMPS voltage registers sit on the SmartReflex
 * I2C bus, which only the PRM voltage controller drives, so no PMIC
 * regulator driver can do this.
 *
 * The voltages offered are the domain's voltage table (volt_data). A change
 * goes through the domain's scale method, which on OMAP4 is the VP force
 * update of the TRM OPP change procedure (OMAP4470 TRM Tables 3-364 and
 * 3-365). The scale method also moves the VC ON command to the new voltage,
 * so a wakeup from retention restores it.
 *
 * An optional "vbb-supply" is the domain's ABB LDO. It is set to the same
 * voltage, after the SMPS when raising and before it when lowering, as in
 * the same TRM tables.
 */

#include <linux/init.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

#include "voltage.h"

struct omap_vdd {
	struct voltagedomain *voltdm;
	struct regulator *vbb;
};

static int omap_vdd_set_voltage_sel(struct regulator_dev *rdev,
				    unsigned int sel)
{
	struct omap_vdd *vdd = rdev_get_drvdata(rdev);
	unsigned int uv = rdev->desc->volt_table[sel];
	bool down = uv < voltdm_get_voltage(vdd->voltdm);
	int ret;

	if (vdd->vbb && down) {
		ret = regulator_set_voltage(vdd->vbb, uv, uv);
		if (ret)
			return ret;
	}

	ret = voltdm_scale(vdd->voltdm, uv);
	if (ret)
		return ret;

	if (vdd->vbb && !down)
		ret = regulator_set_voltage(vdd->vbb, uv, uv);

	return ret;
}

static int omap_vdd_get_voltage(struct regulator_dev *rdev)
{
	struct omap_vdd *vdd = rdev_get_drvdata(rdev);

	return voltdm_get_voltage(vdd->voltdm) ? : -EINVAL;
}

static const struct regulator_ops omap_vdd_ops = {
	.list_voltage		= regulator_list_voltage_table,
	.map_voltage		= regulator_map_voltage_iterate,
	.set_voltage_sel	= omap_vdd_set_voltage_sel,
	.get_voltage		= omap_vdd_get_voltage,
};

static int omap_vdd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regulator_config config = { };
	struct regulator_desc *desc;
	struct regulator_dev *rdev;
	struct voltagedomain *voltdm;
	struct omap_vdd *vdd;
	int ret;
	unsigned int *table;
	const char *name;
	int i, n;

	if (of_property_read_string(dev->of_node, "ti,voltdm", &name))
		return dev_err_probe(dev, -EINVAL, "missing ti,voltdm\n");

	voltdm = voltdm_lookup(name);
	if (!voltdm || !voltdm->scale || !voltdm->pmic ||
	    !voltdm->volt_data || !voltdm->vc_param)
		return dev_err_probe(dev, -ENODEV,
				     "vdd_%s cannot be scaled\n", name);

	for (n = 0; voltdm->volt_data[n].volt_nominal; n++)
		;

	table = devm_kcalloc(dev, n, sizeof(*table), GFP_KERNEL);
	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	vdd = devm_kzalloc(dev, sizeof(*vdd), GFP_KERNEL);
	if (!table || !desc || !vdd)
		return -ENOMEM;
	vdd->voltdm = voltdm;

	for (i = 0; i < n; i++)
		table[i] = voltdm->volt_data[i].volt_nominal;

	/*
	 * Nothing has scaled the domain yet. The voltage layer sets the VC ON
	 * command to the voltage the bootloader left on the rail, and the
	 * SMPS voltage register cannot be read back, so start from that.
	 */
	if (!voltdm->nominal_volt)
		voltdm->nominal_volt = voltdm->vc_param->on;

	vdd->vbb = devm_regulator_get_optional(dev, "vbb");
	if (IS_ERR(vdd->vbb)) {
		if (PTR_ERR(vdd->vbb) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(vdd->vbb),
					     "vbb-supply\n");
		vdd->vbb = NULL;
	} else {
		/* Bring the ABB in line with the starting voltage */
		ret = regulator_set_voltage(vdd->vbb, voltdm->nominal_volt,
					    voltdm->nominal_volt);
		if (ret)
			return dev_err_probe(dev, ret, "vbb at %u uV\n",
					     voltdm->nominal_volt);
	}

	desc->name = devm_kasprintf(dev, GFP_KERNEL, "vdd_%s", name);
	desc->type = REGULATOR_VOLTAGE;
	desc->owner = THIS_MODULE;
	desc->ops = &omap_vdd_ops;
	desc->volt_table = table;
	desc->n_voltages = n;

	config.dev = dev;
	config.of_node = dev->of_node;
	config.driver_data = vdd;
	config.init_data = of_get_regulator_init_data(dev, dev->of_node, desc);
	if (!config.init_data)
		return -ENOMEM;

	rdev = devm_regulator_register(dev, desc, &config);
	if (IS_ERR(rdev))
		return dev_err_probe(dev, PTR_ERR(rdev), "register failed\n");

	dev_info(dev, "vdd_%s at %u uV, %d voltages%s\n", name,
		 voltdm->nominal_volt, n, vdd->vbb ? ", with ABB" : "");

	return 0;
}

static const struct of_device_id omap_vdd_of_match[] = {
	{ .compatible = "ti,omap-voltdm-regulator" },
	{ }
};

static struct platform_driver omap_vdd_driver = {
	.probe = omap_vdd_probe,
	.driver = {
		.name = "omap-voltdm-regulator",
		.of_match_table = omap_vdd_of_match,
		.suppress_bind_attrs = true,
	},
};

/*
 * Registered by the PM code once the voltage layer is initialized, which
 * happens at late_initcall_sync after the PMIC has probed. Probing then
 * binds the regulator at once, and that retries the deferred cpufreq-dt.
 */
int __init omap_vdd_regulator_init(void)
{
	return platform_driver_register(&omap_vdd_driver);
}
