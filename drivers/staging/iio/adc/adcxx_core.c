/*
 * adcxx SPI ADC driver
 *
 * Copyright 2012 WatchFrog Inc.
 *
 * Currently only supports ADC122S101
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/module.h>

#include "../iio.h"
#include "../sysfs.h"

#ifdef CONFIG_IIO_BUFFER
#include "../buffer_generic.h"
#include "../ring_sw.h"
#endif

#ifdef CONFIG_IIO_TRIGGER
#include "../trigger.h"
#include "../trigger_consumer.h"
#endif

#include "adcxx.h"

static int adcxx_scan_direct(struct adcxx_state *st, unsigned ch)
{
	int ret;
	
	printk("%s\n", __func__);
	ret = spi_sync(st->spi, &st->msg[ch]);
	if (ret)
		return ret;

	return (st->data[(ch * 2)] << 8) | st->data[(ch * 2) + 1];
}

static int adcxx_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	int ret;
	struct adcxx_state *st = iio_priv(indio_dev);
	unsigned int scale_uv;

	printk("%s channel %d, address %ld\n", __func__, chan->channel, chan->address);
	switch (m) {
	case 0:
		mutex_lock(&indio_dev->mlock);
#ifdef CONFIG_IIO_BUFFER
		if (iio_buffer_enabled(indio_dev))
			ret = adcxx_scan_from_ring(st, 1 << chan->address);
		else
			ret = adcxx_scan_direct(st, chan->address);
#else
		ret = adcxx_scan_direct(st, chan->address);
#endif
		mutex_unlock(&indio_dev->mlock);

		if (ret < 0)
			return ret;
		*val = (ret >> st->chip_info->channel[0].scan_type.shift) &
			RES_MASK(st->chip_info->channel[0].scan_type.realbits);
		return IIO_VAL_INT;
	case (1 << IIO_CHAN_INFO_SCALE_SHARED):
		scale_uv = (st->int_vref_mv * 1000)
			>> st->chip_info->channel[0].scan_type.realbits;
		*val =  scale_uv/1000;
		*val2 = (scale_uv%1000)*1000;
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}


static const struct adcxx_chip_info adcxx_chip_info_tbl[] = {
	/*
	 * More devices added in future
	 */
	[ID_ADCXX] = {
		.channel[0] = {
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = 0,
			.info_mask = (1 << IIO_CHAN_INFO_SCALE_SHARED),
			.address = 0,
			.scan_index = 0,
			.scan_type = IIO_ST('u', 12, 16, 0),
		},
		.channel[1] = {
			.type = IIO_VOLTAGE,
			.indexed = 1,
			.channel = 1,
			.info_mask = (1 << IIO_CHAN_INFO_SCALE_SHARED),
			.address = 1 << 3,
			.scan_index = 1,
			.scan_type = IIO_ST('u', 12, 16, 0),
		},
		.channel[2] = IIO_CHAN_SOFT_TIMESTAMP(2),
		.int_vref_mv = 3300,
	},
};

static const struct iio_info adcxx_info = {
	.read_raw = &adcxx_read_raw,
	.driver_module = THIS_MODULE,
};

static int __devinit adcxx_probe(struct spi_device *spi)
{
	struct adcxx_platform_data *pdata = spi->dev.platform_data;
	struct adcxx_state *st;
	int ret, voltage_uv = 0;
	struct iio_dev *indio_dev = iio_allocate_device(sizeof(*st));

	printk("%s\n", __func__);
	if (indio_dev == NULL)
	{
		printk("%s: indio_dev is NULL\n", __func__);
		return -ENOMEM;
	}
	st = iio_priv(indio_dev);

#if 0
	st->reg = regulator_get(&spi->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			goto error_put_reg;

		voltage_uv = regulator_get_voltage(st->reg);
	}
#else
	voltage_uv = 3300;
#endif
	st->chip_info =
		&adcxx_chip_info_tbl[spi_get_device_id(spi)->driver_data];

	spi_set_drvdata(spi, indio_dev);
	st->spi = spi;

	/* Estabilish that the iio_dev is a child of the spi device */
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &adcxx_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	/* Setup message to read ch0 */

	st->tx_cmd_buf[0] = ADCXX_CH_AIN0;

	st->xfer[0].rx_buf = &st->data[0];
	st->xfer[0].tx_buf = &st->tx_cmd_buf[0];
	st->xfer[0].len = 2;

	spi_message_init(&st->msg[ADCXX_CH0]);
	spi_message_add_tail(&st->xfer[0], &st->msg[ADCXX_CH0]);

	/* Setup message to read ch1 */

	st->tx_cmd_buf[2] = ADCXX_CH_AIN1;
	st->xfer[1].rx_buf = &st->data[2];
	st->xfer[1].tx_buf = &st->tx_cmd_buf[2];
	st->xfer[1].len = 2;

	spi_message_init(&st->msg[ADCXX_CH1]);
	spi_message_add_tail(&st->xfer[1], &st->msg[ADCXX_CH1]);

	if (pdata && pdata->vref_mv)
		st->int_vref_mv = pdata->vref_mv;
	else if (voltage_uv)
		st->int_vref_mv = voltage_uv / 1000;
	else
		dev_warn(&spi->dev, "reference voltage unspecified\n");

	indio_dev->channels = st->chip_info->channel;
	indio_dev->num_channels = 3;

#ifdef CONFIG_IIO_BUFFER
	ret = adcxx_register_ring_funcs_and_init(indio_dev);
	if (ret)
		goto error_disable_reg;

	ret = iio_buffer_register(indio_dev,
				  indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto error_cleanup_ring;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_unregister_ring;
#else
	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;
#endif
	printk("%s: Done\n", __func__);
	return 0;
#ifdef CONFIG_IIO_BUFFER
error_unregister_ring:
	printk("%s: error_unregister_ring\n", __func__);
	iio_buffer_unregister(indio_dev);
error_cleanup_ring:
	printk("%s: error_cleanup_ring\n", __func__);
	adcxx_ring_cleanup(indio_dev);
#endif
error_disable_reg:
	printk("%s: error_disable_reg\n", __func__);
#if 0
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);
error_put_reg:
	if (!IS_ERR(st->reg))
		regulator_put(st->reg);
#endif
	iio_free_device(indio_dev);

	printk("%s: error %d\n", __func__, ret);

	return ret;
}

static int adcxx_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
#if 0
	struct adcxx_state *st = iio_priv(indio_dev);
#endif

	iio_device_unregister(indio_dev);
#ifdef CONFIG_IIO_BUFFER
	iio_buffer_unregister(indio_dev);
	adcxx_ring_cleanup(indio_dev);
#endif
#if 0
	if (!IS_ERR(st->reg)) {
		regulator_disable(st->reg);
		regulator_put(st->reg);
	}
#endif
	iio_free_device(indio_dev);

	return 0;
}

static const struct spi_device_id adcxx_id[] = {
	{"adcxx", ID_ADCXX},
	{}
};

static struct spi_driver adcxx_driver = {
	.driver = {
		.name	= "adcxx",
		.owner	= THIS_MODULE,
	},
	.probe		= adcxx_probe,
	.remove		= __devexit_p(adcxx_remove),
	.id_table	= adcxx_id,
};

static int __init adcxx_init(void)
{
	return spi_register_driver(&adcxx_driver);
}
module_init(adcxx_init);

static void __exit adcxx_exit(void)
{
	spi_unregister_driver(&adcxx_driver);
}
module_exit(adcxx_exit);

MODULE_AUTHOR("Don Smyth <don@watchfrog.co>");
MODULE_DESCRIPTION("Texas Instruments adcxx ADC");
MODULE_LICENSE("GPL v2");
