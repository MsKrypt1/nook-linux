// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#define OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_0 0x064

static const u32 sda_masks[4] = {
	BIT(16), // I2C1
	BIT(20), // I2C2
	BIT(24), // I2C3
	BIT(28), // I2C4
};

static const u32 scl_masks[4] = {
	BIT(0),  // I2C1
	BIT(4),  // I2C2
	BIT(8),  // I2C3
	BIT(12), // I2C4
};

static int omap4_i2c_pullup_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap *regmap;
	u32 bus_id;
	bool disable_pullups;
	u32 mask, val;
	int ret;

	// Get regmap from syscon phandle
	regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "syscon");
	if (IS_ERR(regmap)) {
		dev_err(dev, "Failed to lookup syscon regmap\n");
		return PTR_ERR(regmap);
	}

	// Get bus-id (1 to 4)
	ret = of_property_read_u32(dev->of_node, "i2c-bus-id", &bus_id);
	if (ret || bus_id < 1 || bus_id > 4) {
		dev_err(dev, "Invalid or missing i2c-bus-id (1-4)\n");
		return -EINVAL;
	}

	// Get boolean flag
	disable_pullups = of_property_read_bool(dev->of_node, "pullups-disabled");

	// Build mask
	mask = sda_masks[bus_id - 1] | scl_masks[bus_id - 1];

	ret = regmap_read(regmap, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_0, &val);
	if (ret) {
		dev_err(dev, "Failed to read register\n");
		return ret;
	}

	if (disable_pullups)
		val |= mask;
	else
		val &= ~mask;

	ret = regmap_write(regmap, OMAP4_CTRL_MODULE_PAD_CORE_CONTROL_I2C_0, val);
	if (ret) {
		dev_err(dev, "Failed to write pullup config\n");
	} else {
		dev_info(dev, "I2C%d pullups %s\n", bus_id,
			 disable_pullups ? "disabled" : "enabled");
	}

	return ret;
}

static const struct of_device_id omap4_i2c_pullup_of_match[] = {
	{ .compatible = "ti,omap4-i2c-pullup" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, omap4_i2c_pullup_of_match);

static struct platform_driver omap4_i2c_pullup_driver = {
	.probe = omap4_i2c_pullup_probe,
	.driver = {
		.name = "omap4-i2c-pullup",
		.of_match_table = omap4_i2c_pullup_of_match,
	},
};

module_platform_driver(omap4_i2c_pullup_driver);

MODULE_AUTHOR("MSKrypt1");
MODULE_DESCRIPTION("OMAP4 I2C Pull-up Control Driver");
MODULE_LICENSE("GPL");