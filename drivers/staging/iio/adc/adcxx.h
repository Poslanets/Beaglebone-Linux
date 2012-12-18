/*
 * ADCXX SPI ADC driver
 *
 * Copyright 2012 WatchFrog Inc.
 *
 * Currently only supports ADC122S101
 *
 * Licensed under the GPL-2 or later.
 */
#ifndef IIO_ADC_ADCXX_H_
#define IIO_ADC_ADCXX_H_

/* control register bits */
#define ADCXX_CH_AIN1		(1 << 3) /* convert on channel 1 */
#define ADCXX_CH_AIN0		(0 << 3) /* convert on channel 0 */

enum adcxx_channels {
	ADCXX_CH0,
	ADCXX_CH1,
};

#define RES_MASK(bits)	((1 << (bits)) - 1) /* TODO: move this into a common header */

/*
 * TODO: struct adcxx_platform_data needs to go into include/linux/iio
 */

struct adcxx_platform_data {
	/* External Vref voltage applied */
	u16				vref_mv;
};

/**
 * struct adcxx_chip_info - chip specifc information
 * @int_vref_mv:	the internal reference voltage
 * @channel:		channel specification
 */

struct adcxx_chip_info {
	u16				int_vref_mv;
	struct iio_chan_spec		channel[3];
};

struct adcxx_state {
	struct spi_device		*spi;
	const struct adcxx_chip_info	*chip_info;
#if 0
	struct regulator		*reg;
#endif
	size_t				d_size;
	u16				int_vref_mv;
	struct spi_transfer		xfer[2];
	struct spi_message		msg[2];
	struct spi_message		*ring_msg;
	unsigned char			tx_cmd_buf[8];

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */

	unsigned char			data[4] ____cacheline_aligned;
};

enum adcxx_supported_device_ids {
	ID_ADCXX
};

#ifdef CONFIG_IIO_BUFFER
int adcxx_scan_from_ring(struct adcxx_state *st, int channum);
int adcxx_register_ring_funcs_and_init(struct iio_dev *indio_dev);
void adcxx_ring_cleanup(struct iio_dev *indio_dev);
#endif /* CONFIG_IIO_BUFFER */
#endif /* IIO_ADC_ADCXX_H_ */
