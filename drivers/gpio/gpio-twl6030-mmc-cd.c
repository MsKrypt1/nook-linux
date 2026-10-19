// SPDX-License-Identifier: GPL-2.0
/*
 * SD card detect of the TWL6030/TWL6032 as a GPIO
 *
 * The PMIC senses the MMC1 card-detect switch: STS_MMC in MMCCTRL is the
 * card state, the MMC interrupt fires on a change. One input line with
 * that interrupt lets the MMC host use it as its cd-gpios.
 */

#include <linux/gpio/driver.h>
#include <linux/mfd/twl.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>

struct twl6030_mmc_cd {
	struct gpio_chip chip;
	int irq;
};

static int twl6030_mmc_cd_get(struct gpio_chip *chip, unsigned int offset)
{
	u8 val;
	int ret;

	ret = twl_i2c_read_u8(TWL6030_MODULE_ID0, &val, TWL6030_MMCCTRL);
	if (ret < 0)
		return ret;

	return !!(val & STS_MMC);
}

static int twl6030_mmc_cd_get_direction(struct gpio_chip *chip,
					unsigned int offset)
{
	return GPIO_LINE_DIRECTION_IN;
}

static int twl6030_mmc_cd_direction_in(struct gpio_chip *chip,
				       unsigned int offset)
{
	return 0;
}

static int twl6030_mmc_cd_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct twl6030_mmc_cd *cd = gpiochip_get_data(chip);

	return cd->irq;
}

static int twl6030_mmc_cd_probe(struct platform_device *pdev)
{
	struct twl6030_mmc_cd *cd;
	int ret;

	cd = devm_kzalloc(&pdev->dev, sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	cd->irq = platform_get_irq(pdev, 0);
	if (cd->irq < 0)
		return cd->irq;

	/*
	 * Unmask the MMC interrupt, card state by switch polarity (SW_FC),
	 * VMMC left to its regulator (no VMMC_AUTO_OFF), no internal pulls:
	 * as B&N's kernel set it up for the Nook HD+.
	 */
	ret = twl6030_mmc_card_detect_config();
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "can't set up MMCCTRL\n");

	cd->chip.label = "twl6030-mmc-cd";
	cd->chip.owner = THIS_MODULE;
	cd->chip.parent = &pdev->dev;
	cd->chip.base = -1;
	cd->chip.ngpio = 1;
	cd->chip.can_sleep = true;
	cd->chip.get = twl6030_mmc_cd_get;
	cd->chip.get_direction = twl6030_mmc_cd_get_direction;
	cd->chip.direction_input = twl6030_mmc_cd_direction_in;
	cd->chip.to_irq = twl6030_mmc_cd_to_irq;

	return devm_gpiochip_add_data(&pdev->dev, &cd->chip, cd);
}

static const struct of_device_id twl6030_mmc_cd_of_match[] = {
	{ .compatible = "ti,twl6030-mmc-cd" },
	{ }
};
MODULE_DEVICE_TABLE(of, twl6030_mmc_cd_of_match);

static struct platform_driver twl6030_mmc_cd_driver = {
	.driver = {
		.name = "twl6030-mmc-cd",
		.of_match_table = twl6030_mmc_cd_of_match,
	},
	.probe = twl6030_mmc_cd_probe,
};
module_platform_driver(twl6030_mmc_cd_driver);

MODULE_DESCRIPTION("TWL6030 SD card detect as a GPIO");
MODULE_LICENSE("GPL");
