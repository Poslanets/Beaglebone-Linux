/*
 * tps65217_bl.c
 *
 * TPS65217 backlight driver
 *
 * Copyright (C) 2012 Matthias Kaehlcke
 * Author: Matthias Kaehlcke <matthias@kaehlcke.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/mfd/tps65217.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct tps65217_bl {
	struct tps65217 *tps;
	struct device *dev;
	struct backlight_device *bl;
	int on;
};

static int tps65217_bl_enable(struct tps65217_bl *tps65217_bl)
{
	int rc;

	rc = tps65217_set_bits(tps65217_bl->tps, TPS65217_REG_WLEDCTRL1,
			TPS65217_WLEDCTRL1_ISINK_ENABLE,
			TPS65217_WLEDCTRL1_ISINK_ENABLE, TPS65217_PROTECT_NONE);
	if (rc) {
		dev_err(tps65217_bl->dev,
			"failed to enable backlight: %d\n", rc);
		return rc;
	}

	tps65217_bl->on = 1;

	dev_dbg(tps65217_bl->dev, "backlight enabled\n");

	return 0;
}

static int tps65217_bl_disable(struct tps65217_bl *tps65217_bl)
{
	int rc;

	rc = tps65217_clear_bits(tps65217_bl->tps,
				TPS65217_REG_WLEDCTRL1,
				TPS65217_WLEDCTRL1_ISINK_ENABLE,
				TPS65217_PROTECT_NONE);
	if (rc) {
		dev_err(tps65217_bl->dev,
			"failed to disable backlight: %d\n", rc);
		return rc;
	}

	tps65217_bl->on = 0;

	dev_dbg(tps65217_bl->dev, "backlight disabled\n");

	return 0;
}

static int tps65217_bl_update_status(struct backlight_device *bl)
{
	struct tps65217_bl *tps65217_bl = bl_get_data(bl);
	int rc;
	int brightness = bl->props.brightness;

	if (bl->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	if ((bl->props.power != FB_BLANK_UNBLANK) ||
		(bl->props.fb_blank != FB_BLANK_UNBLANK))
		/* framebuffer in low power mode or blanking active */
		brightness = 0;

	if (brightness > 0) {
		rc = tps65217_reg_write(tps65217_bl->tps,
					TPS65217_REG_WLEDCTRL2,
					brightness - 1,
					TPS65217_PROTECT_NONE);
		if (rc) {
			dev_err(tps65217_bl->dev,
				"failed to set brightness level: %d\n", rc);
			return rc;
		}

		dev_dbg(tps65217_bl->dev, "brightness set to %d\n", brightness);

		if (!tps65217_bl->on)
			rc = tps65217_bl_enable(tps65217_bl);
	} else {
		rc = tps65217_bl_disable(tps65217_bl);
	}

	return rc;
}

static int tps65217_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops tps65217_bl_ops = {
	.options	= BL_CORE_SUSPENDRESUME,
	.update_status	= tps65217_bl_update_status,
	.get_brightness	= tps65217_bl_get_brightness
};

static int tps65217_bl_hw_init(struct tps65217_bl *tps65217_bl,
			struct tps65217_bl_pdata *pdata)
{
	int rc;

	rc = tps65217_bl_disable(tps65217_bl);
	if (rc)
		return rc;

	switch (pdata->isel) {
	case TPS65217_BL_ISET1:
		/* select ISET_1 current level */
		rc = tps65217_clear_bits(tps65217_bl->tps,
					TPS65217_REG_WLEDCTRL1,
					TPS65217_WLEDCTRL1_ISEL,
					TPS65217_PROTECT_NONE);
		if (rc) {
			dev_err(tps65217_bl->dev,
				"failed to select ISET1 current level: %d)\n",
				rc);
			return rc;
		}

		dev_dbg(tps65217_bl->dev, "selected ISET1 current level\n");

		break;

	case TPS65217_BL_ISET2:
		/* select ISET2 current level */
		rc = tps65217_set_bits(tps65217_bl->tps, TPS65217_REG_WLEDCTRL1,
				TPS65217_WLEDCTRL1_ISEL,
				TPS65217_WLEDCTRL1_ISEL, TPS65217_PROTECT_NONE);
		if (rc) {
			dev_err(tps65217_bl->dev,
				"failed to select ISET2 current level: %d\n",
				rc);
			return rc;
		}

		dev_dbg(tps65217_bl->dev, "selected ISET2 current level\n");

		break;

	default:
		dev_err(tps65217_bl->dev,
			"invalid value for current level: %d\n", pdata->isel);
		return -EINVAL;
	}

	/* set PWM frequency */
	rc = tps65217_set_bits(tps65217_bl->tps,
			TPS65217_REG_WLEDCTRL1,
			TPS65217_WLEDCTRL1_FDIM_MASK,
			pdata->fdim,
			TPS65217_PROTECT_NONE);
	if (rc) {
		dev_err(tps65217_bl->dev,
			"failed to select PWM dimming frequency: %d\n",
			rc);
		return rc;
	}

	return 0;
}

#ifdef CONFIG_OF
static struct tps65217_bl_pdata *
tps65217_bl_parse_dt(struct tps65217_bl *tps65217_bl)
{
	struct device_node *np = tps65217_bl->dev->of_node;
	struct tps65217_bl_pdata *pdata;
	u32 val;

	pdata = devm_kzalloc(tps65217_bl->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(tps65217_bl->dev, "failed to allocate platform data\n");
		return ERR_PTR(-ENOMEM);
	}

	pdata->isel = TPS65217_BL_ISET1;
	if (!of_property_read_u32(np, "isel", &val)) {
		if (val < TPS65217_BL_ISET1 ||
			val > TPS65217_BL_ISET2) {
			dev_err(tps65217_bl->dev,
				"invalid 'isel' value in the device tree\n");
			goto err_einval;
		}

		pdata->isel = val;
	}

	pdata->fdim = TPS65217_BL_FDIM_200HZ;
	if (!of_property_read_u32(np, "fdim", &val)) {
		switch (val) {
		case 100:
			pdata->fdim = TPS65217_BL_FDIM_100HZ;
			break;

		case 200:
			pdata->fdim = TPS65217_BL_FDIM_200HZ;
			break;

		case 500:
			pdata->fdim = TPS65217_BL_FDIM_500HZ;
			break;

		case 1000:
			pdata->fdim = TPS65217_BL_FDIM_1000HZ;
			break;

		default:
			dev_err(tps65217_bl->dev,
				"invalid 'fdim' value in the device tree\n");
			goto err_einval;
		}
	}

	return pdata;

err_einval:
	devm_kfree(tps65217_bl->dev, pdata);

	return ERR_PTR(-EINVAL);
}
#else
static struct tps65217_bl_pdata *
tps65217_bl_parse_dt(struct tps65217_bl *tps65217_bl)
{
	return NULL;
}
#endif

static int tps65217_bl_probe(struct platform_device *pdev)
{
	int rc;
	struct i2c_client *i2c_client = to_i2c_client(pdev->dev.parent);
	struct tps65217 *tps = i2c_get_clientdata(i2c_client);
	struct tps65217_bl *tps65217_bl;
	struct tps65217_bl_pdata *pdata;
	struct backlight_properties bl_props;

	tps65217_bl = devm_kzalloc(&pdev->dev, sizeof(*tps65217_bl),
				GFP_KERNEL);
	if (tps65217_bl == NULL) {
		dev_err(&pdev->dev, "allocation of struct tps65217_bl failed\n");
		return -ENOMEM;
	}

	tps65217_bl->tps = tps;
	tps65217_bl->dev = &pdev->dev;
	tps65217_bl->on = 0;

	if (tps65217_bl->dev->of_node)
		pdata = tps65217_bl_parse_dt(tps65217_bl);
	else
		pdata = pdev->dev.platform_data;

	if (IS_ERR(pdata))
		return PTR_ERR(pdata);

	rc = tps65217_bl_hw_init(tps65217_bl, pdata);
	if (rc)
		goto err_bl_hw_init;

	memset(&bl_props, 0, sizeof(struct backlight_properties));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.max_brightness = 100;

	tps65217_bl->bl = backlight_device_register(pdev->name,
						tps65217_bl->dev, tps65217_bl,
						&tps65217_bl_ops, &bl_props);
	if (IS_ERR(tps65217_bl->bl)) {
		rc = PTR_ERR(tps65217_bl->bl);
		dev_err(tps65217_bl->dev,
			"registration of backlight device failed: %d\n", rc);
		goto err_reg_bl;
	}

	// Set the default brightness to 50%
	tps65217_bl->bl->props.brightness = 50;

	return 0;

err_reg_bl:
err_bl_hw_init:
	return rc;
}

static int tps65217_bl_remove(struct platform_device *pdev)
{
	struct tps65217_bl *tps65217_bl = platform_get_drvdata(pdev);

	backlight_device_unregister(tps65217_bl->bl);

	return 0;
}

static struct platform_driver tps65217_bl_driver = {
	.probe		= tps65217_bl_probe,
	.remove		= tps65217_bl_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65217-bl",
	},
};

static int __init tps65217_bl_init(void)
{
	return platform_driver_register(&tps65217_bl_driver);
}

static void __exit tps65217_bl_exit(void)
{
	platform_driver_unregister(&tps65217_bl_driver);
}

module_init(tps65217_bl_init);
module_exit(tps65217_bl_exit);

MODULE_DESCRIPTION("TPS65217 Backlight driver");
MODULE_LICENSE("GPLv2");
MODULE_AUTHOR("Matthias Kaehlcke <matthias@kaehlcke.net>");
