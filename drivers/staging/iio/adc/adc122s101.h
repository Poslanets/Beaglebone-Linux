/*
 * ADC122S101 SPI ADC driver
 *
 * Copyright 2012 WatchFrog Inc.
 *
 * Currently only supports ADC122S101
 *
 * Licensed under the GPL-2 or later.
 */
#ifndef IIO_ADC_ADC122S101_H_
#define IIO_ADC_ADC122S101_H_

/* control register bits */
#define ADC122S101_CH_AIN1		(1 << 3) /* convert on channel 1 */
#define ADC122S101_CH_AIN0		(0 << 3) /* convert on channel 0 */

#define RES_MASK(bits)	((1 << (bits)) - 1) /* TODO: move this into a common header */

/*
 * TODO: struct adc122s101_platform_data needs to go into include/linux/iio
 */

struct adc122s101_platform_data {
	/* External Vref voltage applied */
	u16				vref_mv;
};

/**
 * struct adc122s101_chip_info - chip specifc information
 * @int_vref_mv:	the internal reference voltage
 * @channel:		channel specification
 */

struct adc122s101_chip_info {
	u16				int_vref_mv;
	struct iio_chan_spec		channel[3];
};

struct adc122s101_state {
	struct spi_device		*spi;
	struct iio_trigger		*trig;
	const struct adc122s101_chip_info	*chip_info;
#if 0
	struct regulator		*reg;
#endif
	wait_queue_head_t		wq_data_avail;
	bool				done;
	bool				irq_dis;
	size_t				d_size;
	u16				int_vref_mv;
	/* need three messages for chan0, chan1 and both */
	struct spi_transfer		xfer[3];
	struct spi_message		msg[3];
	struct spi_message		*ring_msg;
	unsigned char			tx_cmd_buf[8];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */

	unsigned char			data[4] ____cacheline_aligned;
};

enum adc122s101_supported_device_ids {
	ID_ADC122S101
};

#ifdef CONFIG_IIO_BUFFER
int adc122s101_scan_from_ring(struct adc122s101_state *st, int channum);
int adc122s101_register_ring_funcs_and_init(struct iio_dev *indio_dev);
void adc122s101_ring_cleanup(struct iio_dev *indio_dev);
int adc122s101_probe_trigger(struct iio_dev *indio_dev);
void adc122s101_remove_trigger(struct iio_dev *indio_dev);
#endif /* CONFIG_IIO_BUFFER */
#endif /* IIO_ADC_ADC122S101_H_ */
