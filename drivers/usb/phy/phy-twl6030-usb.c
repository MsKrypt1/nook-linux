// SPDX-License-Identifier: GPL-2.0+
/*
 * twl6030_usb - TWL6030 USB transceiver, talking to OMAP OTG driver.
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - https://www.ti.com
 *
 * Author: Hema HK <hemahk@ti.com>
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/phy/phy.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/usb/musb.h>
#include <linux/usb/phy_companion.h>
#include <linux/phy/omap_usb.h>
#include <linux/mfd/twl.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>

/* usb register definitions */
#define USB_VENDOR_ID_LSB		0x00
#define USB_VENDOR_ID_MSB		0x01
#define USB_PRODUCT_ID_LSB		0x02
#define USB_PRODUCT_ID_MSB		0x03
#define USB_VBUS_CTRL_SET		0x04
#define USB_VBUS_CTRL_CLR		0x05
#define USB_ID_CTRL_SET			0x06
#define USB_ID_CTRL_CLR			0x07
#define USB_VBUS_INT_SRC		0x08
#define USB_VBUS_INT_LATCH_SET		0x09
#define USB_VBUS_INT_LATCH_CLR		0x0A
#define USB_VBUS_INT_EN_LO_SET		0x0B
#define USB_VBUS_INT_EN_LO_CLR		0x0C
#define USB_VBUS_INT_EN_HI_SET		0x0D
#define USB_VBUS_INT_EN_HI_CLR		0x0E
#define USB_ID_INT_SRC			0x0F
#define USB_ID_INT_LATCH_SET		0x10
#define USB_ID_INT_LATCH_CLR		0x11

#define USB_ID_INT_EN_LO_SET		0x12
#define USB_ID_INT_EN_LO_CLR		0x13
#define USB_ID_INT_EN_HI_SET		0x14
#define USB_ID_INT_EN_HI_CLR		0x15
#define USB_OTG_ADP_CTRL		0x16
#define USB_OTG_ADP_HIGH		0x17
#define USB_OTG_ADP_LOW			0x18
#define USB_OTG_ADP_RISE		0x19
#define USB_OTG_REVISION		0x1A

/* to be moved to LDO */
#define TWL6030_MISC2			0xE5
#define TWL6030_CFG_LDO_PD2		0xF5
#define TWL6030_BACKUP_REG		0xFA

#define STS_HW_CONDITIONS		0x21

/* In module TWL6030_MODULE_PM_MASTER */
#define STS_HW_CONDITIONS		0x21
#define STS_USB_ID			BIT(2)

/* In module TWL6030_MODULE_PM_RECEIVER */
#define VUSB_CFG_TRANS			0x71
#define VUSB_CFG_STATE			0x72
#define VUSB_CFG_VOLTAGE		0x73

/* in module TWL6030_MODULE_MAIN_CHARGE */

#define CHARGERUSB_CTRL1		0x8

#define CONTROLLER_STAT1		0x03
#define	VBUS_DET			BIT(2)

/* OMAP4 CONTROL_USB2PHYCORE: the USB2 PHY's charger detection */
#define USB2PHY_DISCHGDET		BIT(30)
#define USB2PHY_CHG_DET_STATUS		GENMASK(23, 21)
#define USB2PHY_CHG_DET_WAIT		0
#define USB2PHY_CHG_DET_DEDICATED	4
#define USB2PHY_CHG_DET_HOST		5
#define USB2PHY_CHG_DET_PC		6
#define USB2PHY_RESTARTCHGDET		BIT(15)

struct twl6030_usb {
	struct phy_companion	comparator;
	struct device		*dev;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	struct regulator		*usb3v3;

	/* used to check initial cable status after probe */
	struct delayed_work	get_status_work;

	/* used to set vbus, in atomic path */
	struct work_struct	set_vbus_work;

	int			irq1;
	int			irq2;
	enum musb_vbus_id_status linkstat;
	u8			asleep;
	bool			vbus_enable;

	/* charger detection, optional */
	struct phy		*phy;
	struct regmap		*chgdet;
	unsigned int		chgdet_reg;
	struct power_supply	*psy;
	enum power_supply_usb_type usb_type;
};

#define	comparator_to_twl(x) container_of((x), struct twl6030_usb, comparator)

/*-------------------------------------------------------------------------*/

static inline int twl6030_writeb(struct twl6030_usb *twl, u8 module,
						u8 data, u8 address)
{
	int ret = 0;

	ret = twl_i2c_write_u8(module, data, address);
	if (ret < 0)
		dev_err(twl->dev,
			"Write[0x%x] Error %d\n", address, ret);
	return ret;
}

static inline u8 twl6030_readb(struct twl6030_usb *twl, u8 module, u8 address)
{
	u8 data;
	int ret;

	ret = twl_i2c_read_u8(module, &data, address);
	if (ret >= 0)
		ret = data;
	else
		dev_err(twl->dev,
			"readb[0x%x,0x%x] Error %d\n",
					module, address, ret);
	return ret;
}

static int twl6030_start_srp(struct phy_companion *comparator)
{
	struct twl6030_usb *twl = comparator_to_twl(comparator);

	twl6030_writeb(twl, TWL_MODULE_USB, 0x24, USB_VBUS_CTRL_SET);
	twl6030_writeb(twl, TWL_MODULE_USB, 0x84, USB_VBUS_CTRL_SET);

	mdelay(100);
	twl6030_writeb(twl, TWL_MODULE_USB, 0xa0, USB_VBUS_CTRL_CLR);

	return 0;
}

static int twl6030_usb_ldo_init(struct twl6030_usb *twl)
{
	/* Set to OTG_REV 1.3 and turn on the ID_WAKEUP_COMP */
	twl6030_writeb(twl, TWL6030_MODULE_ID0, 0x1, TWL6030_BACKUP_REG);

	/* Program CFG_LDO_PD2 register and set VUSB bit */
	twl6030_writeb(twl, TWL6030_MODULE_ID0, 0x1, TWL6030_CFG_LDO_PD2);

	/* Program MISC2 register and set bit VUSB_IN_VBAT */
	twl6030_writeb(twl, TWL6030_MODULE_ID0, 0x10, TWL6030_MISC2);

	twl->usb3v3 = regulator_get(twl->dev, "usb");
	if (IS_ERR(twl->usb3v3))
		return -ENODEV;

	/* Program the USB_VBUS_CTRL_SET and set VBUS_ACT_COMP bit */
	twl6030_writeb(twl, TWL_MODULE_USB, 0x4, USB_VBUS_CTRL_SET);

	/*
	 * Program the USB_ID_CTRL_SET register to enable GND drive
	 * and the ID comparators
	 */
	twl6030_writeb(twl, TWL_MODULE_USB, 0x14, USB_ID_CTRL_SET);

	return 0;
}

static ssize_t vbus_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct twl6030_usb *twl = dev_get_drvdata(dev);
	unsigned long flags;
	int ret = -EINVAL;

	spin_lock_irqsave(&twl->lock, flags);

	switch (twl->linkstat) {
	case MUSB_VBUS_VALID:
	       ret = sysfs_emit(buf, "vbus\n");
	       break;
	case MUSB_ID_GROUND:
	       ret = sysfs_emit(buf, "id\n");
	       break;
	case MUSB_VBUS_OFF:
	       ret = sysfs_emit(buf, "none\n");
	       break;
	default:
	       ret = sysfs_emit(buf, "UNKNOWN\n");
	}
	spin_unlock_irqrestore(&twl->lock, flags);

	return ret;
}
static DEVICE_ATTR_RO(vbus);

static struct attribute *twl6030_attrs[] = {
	&dev_attr_vbus.attr,
	NULL,
};
ATTRIBUTE_GROUPS(twl6030);

/*
 * BC1.2 detection by the OMAP4 USB2 PHY, as in TI's 3.0 kernel.  It runs
 * while the PHY is powered; restart it for this VBUS session.  Run it
 * before musb sees VBUS, as BC1.2 wants detection before connecting.
 */
static void twl6030_usb_charger_detect(struct twl6030_usb *twl)
{
	enum power_supply_usb_type type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	unsigned int v = 0;
	int ret;

	ret = phy_init(twl->phy);
	if (ret)
		goto out;
	ret = phy_power_on(twl->phy);
	if (ret)
		goto exit;

	regmap_update_bits(twl->chgdet, twl->chgdet_reg,
			   USB2PHY_DISCHGDET | USB2PHY_RESTARTCHGDET,
			   USB2PHY_RESTARTCHGDET);
	usleep_range(2000, 3000);
	regmap_clear_bits(twl->chgdet, twl->chgdet_reg, USB2PHY_RESTARTCHGDET);

	ret = regmap_read_poll_timeout(twl->chgdet, twl->chgdet_reg, v,
			FIELD_GET(USB2PHY_CHG_DET_STATUS, v) !=
			USB2PHY_CHG_DET_WAIT, 20000, 2000000);
	if (!ret) {
		switch (FIELD_GET(USB2PHY_CHG_DET_STATUS, v)) {
		case USB2PHY_CHG_DET_DEDICATED:
			type = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case USB2PHY_CHG_DET_HOST:
			type = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		case USB2PHY_CHG_DET_PC:
			type = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		}
	}

	phy_power_off(twl->phy);
exit:
	phy_exit(twl->phy);
out:
	if (ret)
		dev_warn(twl->dev, "charger detection failed: %d\n", ret);
	dev_dbg(twl->dev, "charger detection: USB2PHYCORE 0x%08x\n", v);
	twl->usb_type = type;
}

static int twl6030_usb_psy_get_prop(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct twl6030_usb *twl = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = twl->linkstat == MUSB_VBUS_VALID;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = twl->linkstat == MUSB_VBUS_VALID ?
			      twl->usb_type : POWER_SUPPLY_USB_TYPE_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (twl->linkstat != MUSB_VBUS_VALID)
			val->intval = 500000;
		else if (twl->usb_type == POWER_SUPPLY_USB_TYPE_DCP)
			val->intval = 2000000;
		else if (twl->usb_type == POWER_SUPPLY_USB_TYPE_CDP)
			val->intval = 1500000;
		else
			val->intval = 500000;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const enum power_supply_property twl6030_usb_psy_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static const struct power_supply_desc twl6030_usb_psy_desc = {
	.name = "twl6030-usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
	.properties = twl6030_usb_psy_props,
	.num_properties = ARRAY_SIZE(twl6030_usb_psy_props),
	.get_property = twl6030_usb_psy_get_prop,
};

static int twl6030_usb_charger_init(struct twl6030_usb *twl)
{
	struct power_supply_config psy_cfg = {
		.drv_data = twl,
		.fwnode = dev_fwnode(twl->dev),
	};
	struct device *dev = twl->dev;
	unsigned int args;

	twl->chgdet = syscon_regmap_lookup_by_phandle_args(dev->of_node,
						"syscon-chgdet", 1, &args);
	if (IS_ERR(twl->chgdet)) {
		if (PTR_ERR(twl->chgdet) == -ENOENT)
			return 0;
		return dev_err_probe(dev, PTR_ERR(twl->chgdet),
				     "Can't get syscon-chgdet\n");
	}
	twl->chgdet_reg = args;

	twl->phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(twl->phy))
		return dev_err_probe(dev, PTR_ERR(twl->phy),
				     "Can't get the USB2 PHY\n");

	twl->psy = devm_power_supply_register(dev, &twl6030_usb_psy_desc,
					      &psy_cfg);
	if (IS_ERR(twl->psy))
		return dev_err_probe(dev, PTR_ERR(twl->psy),
				     "Can't register power supply\n");

	return 0;
}

/*
 * Hold one LDOUSB reference while VBUS or ID is present.  A plug often
 * raises more than one VBUS interrupt, and each used to take another.
 */
static void twl6030_usb_ldo_get(struct twl6030_usb *twl)
{
	if (twl->asleep)
		return;

	if (regulator_enable(twl->usb3v3))
		dev_err(twl->dev, "Failed to enable usb3v3\n");
	else
		twl->asleep = 1;
}

static irqreturn_t twl6030_usb_irq(int irq, void *_twl)
{
	struct twl6030_usb *twl = _twl;
	enum musb_vbus_id_status status = MUSB_UNKNOWN;
	u8 vbus_state, hw_state;
	int ret;

	hw_state = twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);

	vbus_state = twl6030_readb(twl, TWL_MODULE_MAIN_CHARGE,
						CONTROLLER_STAT1);
	/* A plug or unplug often raises more than one interrupt */
	if (!(hw_state & STS_USB_ID)) {
		if (vbus_state & VBUS_DET) {
			if (twl->linkstat == MUSB_VBUS_VALID)
				goto out;

			twl6030_usb_ldo_get(twl);

			if (twl->psy)
				twl6030_usb_charger_detect(twl);

			status = MUSB_VBUS_VALID;
			twl->linkstat = status;
			ret = musb_mailbox(status);
			if (ret)
				twl->linkstat = MUSB_UNKNOWN;
		} else {
			if (twl->linkstat != MUSB_UNKNOWN &&
			    twl->linkstat != MUSB_VBUS_OFF) {
				status = MUSB_VBUS_OFF;
				twl->linkstat = status;
				ret = musb_mailbox(status);
				if (ret)
					twl->linkstat = MUSB_UNKNOWN;
				if (twl->asleep) {
					regulator_disable(twl->usb3v3);
					twl->asleep = 0;
				}
			}
		}
	}
	sysfs_notify(&twl->dev->kobj, NULL, "vbus");
	if (twl->psy && status != MUSB_UNKNOWN)
		power_supply_changed(twl->psy);
out:
	return IRQ_HANDLED;
}

static irqreturn_t twl6030_usbotg_irq(int irq, void *_twl)
{
	struct twl6030_usb *twl = _twl;
	enum musb_vbus_id_status status = MUSB_UNKNOWN;
	u8 hw_state;
	int ret;

	hw_state = twl6030_readb(twl, TWL6030_MODULE_ID0, STS_HW_CONDITIONS);

	if (hw_state & STS_USB_ID) {
		twl6030_usb_ldo_get(twl);
		twl6030_writeb(twl, TWL_MODULE_USB, 0x1, USB_ID_INT_EN_HI_CLR);
		twl6030_writeb(twl, TWL_MODULE_USB, 0x10, USB_ID_INT_EN_HI_SET);
		status = MUSB_ID_GROUND;
		twl->linkstat = status;
		ret = musb_mailbox(status);
		if (ret)
			twl->linkstat = MUSB_UNKNOWN;
	} else  {
		twl6030_writeb(twl, TWL_MODULE_USB, 0x10, USB_ID_INT_EN_HI_CLR);
		twl6030_writeb(twl, TWL_MODULE_USB, 0x1, USB_ID_INT_EN_HI_SET);
	}
	twl6030_writeb(twl, TWL_MODULE_USB, status, USB_ID_INT_LATCH_CLR);

	return IRQ_HANDLED;
}

static void twl6030_status_work(struct work_struct *work)
{
	struct twl6030_usb *twl = container_of(work, struct twl6030_usb,
					       get_status_work.work);

	twl6030_usb_irq(twl->irq2, twl);
	twl6030_usbotg_irq(twl->irq1, twl);
}

static int twl6030_enable_irq(struct twl6030_usb *twl)
{
	twl6030_writeb(twl, TWL_MODULE_USB, 0x1, USB_ID_INT_EN_HI_SET);
	twl6030_interrupt_unmask(0x05, REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(0x05, REG_INT_MSK_STS_C);

	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_LINE_C);
	twl6030_interrupt_unmask(TWL6030_CHARGER_CTRL_INT_MASK,
				REG_INT_MSK_STS_C);

	return 0;
}

static void otg_set_vbus_work(struct work_struct *data)
{
	struct twl6030_usb *twl = container_of(data, struct twl6030_usb,
								set_vbus_work);

	/*
	 * Start driving VBUS. Set OPA_MODE bit in CHARGERUSB_CTRL1
	 * register. This enables boost mode.
	 */

	if (twl->vbus_enable)
		twl6030_writeb(twl, TWL_MODULE_MAIN_CHARGE, 0x40,
							CHARGERUSB_CTRL1);
	else
		twl6030_writeb(twl, TWL_MODULE_MAIN_CHARGE, 0x00,
							CHARGERUSB_CTRL1);
}

static int twl6030_set_vbus(struct phy_companion *comparator, bool enabled)
{
	struct twl6030_usb *twl = comparator_to_twl(comparator);

	twl->vbus_enable = enabled;
	schedule_work(&twl->set_vbus_work);

	return 0;
}

static int twl6030_usb_probe(struct platform_device *pdev)
{
	u32 ret;
	struct twl6030_usb	*twl;
	int			status, err;
	struct device_node	*np = pdev->dev.of_node;
	struct device		*dev = &pdev->dev;

	if (!np) {
		dev_err(dev, "no DT info\n");
		return -EINVAL;
	}

	twl = devm_kzalloc(dev, sizeof(*twl), GFP_KERNEL);
	if (!twl)
		return -ENOMEM;

	twl->dev		= &pdev->dev;
	twl->irq1		= platform_get_irq(pdev, 0);
	twl->irq2		= platform_get_irq(pdev, 1);
	twl->linkstat		= MUSB_UNKNOWN;

	if (twl->irq1 < 0)
		return twl->irq1;
	if (twl->irq2 < 0)
		return twl->irq2;

	twl->comparator.set_vbus	= twl6030_set_vbus;
	twl->comparator.start_srp	= twl6030_start_srp;

	ret = omap_usb2_set_comparator(&twl->comparator);
	if (ret == -ENODEV) {
		dev_info(&pdev->dev, "phy not ready, deferring probe");
		return -EPROBE_DEFER;
	}

	/* init spinlock for workqueue */
	spin_lock_init(&twl->lock);

	err = twl6030_usb_ldo_init(twl);
	if (err) {
		dev_err(&pdev->dev, "ldo init failed\n");
		return err;
	}

	platform_set_drvdata(pdev, twl);

	err = twl6030_usb_charger_init(twl);
	if (err)
		goto err_put_regulator_noirq;

	INIT_WORK(&twl->set_vbus_work, otg_set_vbus_work);
	INIT_DELAYED_WORK(&twl->get_status_work, twl6030_status_work);

	status = request_threaded_irq(twl->irq1, NULL, twl6030_usbotg_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"twl6030_usb", twl);
	if (status < 0) {
		dev_err(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq1, status);
		goto err_put_regulator;
	}

	status = request_threaded_irq(twl->irq2, NULL, twl6030_usb_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"twl6030_usb", twl);
	if (status < 0) {
		dev_err(&pdev->dev, "can't get IRQ %d, err %d\n",
			twl->irq2, status);
		goto err_free_irq1;
	}

	twl->asleep = 0;
	twl6030_enable_irq(twl);
	schedule_delayed_work(&twl->get_status_work, HZ);
	dev_info(&pdev->dev, "Initialized TWL6030 USB module\n");

	return 0;

err_free_irq1:
	free_irq(twl->irq1, twl);
err_put_regulator:
	regulator_put(twl->usb3v3);

	return status;

err_put_regulator_noirq:
	regulator_put(twl->usb3v3);

	return err;
}

static void twl6030_usb_remove(struct platform_device *pdev)
{
	struct twl6030_usb *twl = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&twl->get_status_work);
	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK,
		REG_INT_MSK_LINE_C);
	twl6030_interrupt_mask(TWL6030_USBOTG_INT_MASK,
			REG_INT_MSK_STS_C);
	free_irq(twl->irq1, twl);
	free_irq(twl->irq2, twl);
	if (twl->asleep)
		regulator_disable(twl->usb3v3);
	regulator_put(twl->usb3v3);
	cancel_work_sync(&twl->set_vbus_work);
}

static const struct of_device_id twl6030_usb_id_table[] = {
	{ .compatible = "ti,twl6030-usb" },
	{}
};
MODULE_DEVICE_TABLE(of, twl6030_usb_id_table);

static struct platform_driver twl6030_usb_driver = {
	.probe		= twl6030_usb_probe,
	.remove		= twl6030_usb_remove,
	.driver		= {
		.name	= "twl6030_usb",
		.of_match_table = of_match_ptr(twl6030_usb_id_table),
		.dev_groups = twl6030_groups,
	},
};

static int __init twl6030_usb_init(void)
{
	return platform_driver_register(&twl6030_usb_driver);
}
subsys_initcall(twl6030_usb_init);

static void __exit twl6030_usb_exit(void)
{
	platform_driver_unregister(&twl6030_usb_driver);
}
module_exit(twl6030_usb_exit);

MODULE_ALIAS("platform:twl6030_usb");
MODULE_AUTHOR("Hema HK <hemahk@ti.com>");
MODULE_DESCRIPTION("TWL6030 USB transceiver driver");
MODULE_LICENSE("GPL");
