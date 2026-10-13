// SPDX-License-Identifier: GPL-2.0
/*
 * TWL6030/TWL6032 power button driver
 *
 * Reconstructed from the stable ovation build #159
 * (BACKUP/previous-linux-boot.img); the original source was lost.
 */

#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/mfd/twl.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/delay.h>

#define PWR_PWRON_IRQ		BIT(0)
#define STS_HW_CONDITIONS	0x21

struct twl6030_pwr_button {
	struct input_dev *input_dev;
	struct device *dev;
	int report_key;
};

static int twl6030_readb(struct twl6030_pwr_button *pwr, u8 module, u8 address)
{
	u8 data = 0;
	int ret;

	ret = twl_i2c_read_u8(module, &data, address);
	if (ret < 0)
		dev_dbg(pwr->dev, "TWL6030:readb[0x%x,0x%x] Error %d\n",
			module, address, ret);

	return data;
}

static irqreturn_t powerbutton_irq(int irq, void *_pwr)
{
	struct twl6030_pwr_button *pwr = _pwr;
	static int prev_hw_state = 0xffff;
	static int push_release_flag;
	int hw_state, pwr_val;

	hw_state = twl6030_readb(pwr, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);
	pwr_val = !(hw_state & PWR_PWRON_IRQ);

	if (prev_hw_state != pwr_val && prev_hw_state != 0xffff) {
		pm_wakeup_event(pwr->input_dev->dev.parent, 0);
		push_release_flag = 0;
		input_report_key(pwr->input_dev, pwr->report_key, pwr_val);
		input_sync(pwr->input_dev);
	} else if (!push_release_flag) {
		/* Only one edge seen: synthesize the missing press/release */
		push_release_flag = 1;
		input_report_key(pwr->input_dev, pwr->report_key, !pwr_val);
		input_sync(pwr->input_dev);

		msleep(20);

		input_report_key(pwr->input_dev, pwr->report_key, pwr_val);
		input_sync(pwr->input_dev);
	} else {
		push_release_flag = 0;
	}

	prev_hw_state = pwr_val;

	return IRQ_HANDLED;
}

static int twl6030_pwrbutton_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct twl6030_pwr_button *pwr;
	int irq = platform_get_irq(pdev, 0);
	int err;

	dev_dbg(dev, "%s: Enter\n", __func__);

	pwr = devm_kzalloc(dev, sizeof(*pwr), GFP_KERNEL);
	if (!pwr)
		return -ENOMEM;

	pwr->input_dev = devm_input_allocate_device(dev);
	if (!pwr->input_dev) {
		dev_dbg(dev, "Can't allocate power button: %d\n", -ENODEV);
		return -ENODEV;
	}

	input_set_capability(pwr->input_dev, EV_KEY, KEY_POWER);

	pwr->report_key = KEY_POWER;
	pwr->dev = dev;
	pwr->input_dev->evbit[0] = BIT_MASK(EV_KEY);
	pwr->input_dev->keybit[BIT_WORD(pwr->report_key)] =
			BIT_MASK(pwr->report_key);
	pwr->input_dev->name = "twl6030_pwrbutton";
	pwr->input_dev->phys = "twl6030_pwrbutton/input0";
	pwr->input_dev->dev.parent = dev;

	err = devm_request_threaded_irq(dev, irq, NULL, powerbutton_irq,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING |
					IRQF_ONESHOT,
					"twl6030_pwrbutton", pwr);
	if (err < 0) {
		dev_dbg(dev, "Can't get IRQ for pwrbutton: %d\n", err);
		return err;
	}

	err = input_register_device(pwr->input_dev);
	if (err) {
		dev_dbg(dev, "Can't register power button: %d\n", err);
		return err;
	}

	/* The power button is the user's way to resume from suspend */
	err = devm_device_init_wakeup(dev);
	if (err)
		return err;
	err = devm_pm_set_wake_irq(dev, irq);
	if (err)
		return err;

	twl6030_interrupt_unmask(0x01, REG_INT_MSK_LINE_A);
	twl6030_interrupt_unmask(0x01, REG_INT_MSK_STS_A);

	platform_set_drvdata(pdev, pwr);

	return 0;
}

static const struct of_device_id twl6030_pwrbutton_of_match[] = {
	{ .compatible = "ti,twl6030-pwrbutton" },
	{ }
};
MODULE_DEVICE_TABLE(of, twl6030_pwrbutton_of_match);

static struct platform_driver twl6030_pwrbutton_driver = {
	.probe		= twl6030_pwrbutton_probe,
	.driver		= {
		.name	= "twl6030_pwrbutton",
		.of_match_table = twl6030_pwrbutton_of_match,
	},
};
module_platform_driver(twl6030_pwrbutton_driver);

MODULE_ALIAS("platform:twl6030_pwrbutton");
MODULE_DESCRIPTION("TWL6030 Power Button");
MODULE_LICENSE("GPL");
