/*
 * Copyright 2012 WatchFrog Inc.
 *
 * Currently only supports ADC122S101
 *
 * Licensed under the GPL-2.
 *
 * adc122s101_ring.c
 */

#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include "../iio.h"
#include "../buffer_generic.h"
#include "../ring_sw.h"
#include "../trigger_consumer.h"

#include "adc122s101.h"

int adc122s101_scan_from_ring(struct adc122s101_state *st, int channum)
{
	struct iio_buffer *ring = iio_priv_to_dev(st)->buffer;
	int count = 0, ret;
	u16 *ring_data;

	printk("%s: channum 0x%04x, scan_mask 0x%08lx\n", __func__,
			channum, *(ring->scan_mask));
	if (!(test_bit(channum, ring->scan_mask))) {
		printk("%s: channum not in scan_mask\n", __func__);
		ret = -EBUSY;
		goto error_ret;
	}

	printk("%s: bytes_per_datum %d\n", __func__,
			ring->access->get_bytes_per_datum(ring));
	ring_data = kmalloc(ring->access->get_bytes_per_datum(ring), GFP_KERNEL);
	if (ring_data == NULL) {
		printk("%s: can't allocate ring_data\n", __func__);
		ret = -ENOMEM;
		goto error_ret;
	}

	ret = ring->access->read_last(ring, (u8 *) ring_data);
	if (ret) {
		printk("%s: can't read_last ring_data\n", __func__);
		goto error_free_ring_data;
	}

	/* for single channel scan the result is stored with zero offset */
	if ((test_bit(1, ring->scan_mask) || test_bit(0, ring->scan_mask)) &&
	    (channum == 1))
		count = 1;

	ret = be16_to_cpu(ring_data[count]);

	printk("%s: count %d, ret %d\n", __func__,
			count, ret);

error_free_ring_data:
	kfree(ring_data);
error_ret:
	return ret;
}

/**
 * adc122s101_ring_preenable() setup the parameters of the ring before enabling
 *
 * The complex nature of the setting of the number of bytes per datum is due
 * to this driver currently ensuring that the timestamp is stored at an 8
 * byte boundary.
 **/
static int adc122s101_ring_preenable(struct iio_dev *indio_dev)
{
	struct adc122s101_state *st;
	struct iio_buffer *ring;

	printk("%s\n", __func__);
	
	if (indio_dev == NULL)	{
		printk("%s : error NULL iio_dev\n", __func__);
		return -ENOMEM;
	}
	
	st = iio_priv(indio_dev);
	if (st == NULL)	{
		printk("%s : error NULL st\n", __func__);
		return -ENOMEM;
	}

	ring = indio_dev->buffer;
	if (ring == NULL)	{
		printk("%s : error NULL ring\n", __func__);
		return -ENOMEM;
	}
	
	printk("%s : scan_count %d, storagebits %d\n", __func__,
			ring->scan_count,
			st->chip_info->channel[0].scan_type.storagebits);
	
	st->d_size = ring->scan_count *
		st->chip_info->channel[0].scan_type.storagebits / 8;

	printk("%s : d_size %d\n", __func__, st->d_size);
	printk("%s : scan_timestamp %d\n", __func__, ring->scan_timestamp);
	
	if (ring->scan_timestamp) {
		st->d_size += sizeof(s64);

		if (st->d_size % sizeof(s64))
			st->d_size += sizeof(s64) - (st->d_size % sizeof(s64));
		printk("%s : d_size %d\n", __func__, st->d_size);
	}

	if (indio_dev->buffer->access->set_bytes_per_datum)
		indio_dev->buffer->access->set_bytes_per_datum(indio_dev->buffer, st->d_size);
	else
		printk("%s : error setting bytes_per_datum\n", __func__);
	
	/* We know this is a single long so can 'cheat' */
	printk("%s : scan_mask 0x%08x\n", __func__, *ring->scan_mask);
	switch (*ring->scan_mask) {
	/* read channel 0 */
	case (1 << 0):
		printk("%s : scan channel 0\n", __func__);
		st->ring_msg = &st->msg[0];
		break;
	/* read channel 1 */
	case (1 << 1):
		printk("%s : scan channel 1\n", __func__);
		st->ring_msg = &st->msg[1];
		/* Dummy read: push CH1 setting down to hardware */
		spi_sync(st->spi, st->ring_msg);
		break;
	/* read both channels */
	case ((1 << 1) | (1 << 0)):
		printk("%s : scan both channels\n", __func__);
		st->ring_msg = &st->msg[3];
		break;
	default:
		printk("%s : bad scan mask\n", __func__);
		break;
	}

	return 0;
}

static int adc122s101_ring_postdisable(struct iio_dev *indio_dev)
{
	struct adc122s101_state *st = iio_priv(indio_dev);

	printk("%s\n", __func__);

	/* dummy read: restore default CH0 settin */
	return spi_sync(st->spi, &st->msg[0]);
}

/**
 * adc122s101_trigger_handler() bh of trigger launched polling to ring buffer
 *
 * Currently there is no option in this driver to disable the saving of
 * timestamps within the ring.
 **/

static irqreturn_t adc122s101_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct adc122s101_state *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	s64 time_ns;
	__u8 *buf;
	int b_sent;
	unsigned int bytes;
	int i;

	printk("%s\n", __func__);

	bytes = ring->scan_count *
		st->chip_info->channel[0].scan_type.storagebits / 8;

	printk("%s: scan_count %d, storagebits %d, bytes %d, d_size %d\n", __func__,
		ring->scan_count, st->chip_info->channel[0].scan_type.storagebits,
		bytes, st->d_size);

	buf = kzalloc(st->d_size, GFP_KERNEL);
	if (buf == NULL) {
		printk("%s error allocating buf\n", __func__);
		return -ENOMEM;
	}

	printk("%s: spi %p, ring_msg %p\n", __func__, st->spi, st->ring_msg);

	b_sent = spi_sync(st->spi, st->ring_msg);
	if (b_sent)	{
		printk("%s error from spi_sync: %d\n", __func__, b_sent);
		goto done;
	}

	time_ns = iio_get_time_ns();
	memcpy(buf, st->data, bytes);
	if (ring->scan_timestamp) {
		printk("%s: store timestamp\n", __func__);
		memcpy(buf + st->d_size - sizeof(s64),
		       &time_ns, sizeof(time_ns));
	}

	printk("%s: buf is ", __func__);
	for (i=0; i<st->d_size; i++) {
		printk(" 0x%hhx", buf[i]);
	}
	printk("\n");

	indio_dev->buffer->access->store_to(indio_dev->buffer, buf, time_ns);
done:
	printk("%s: done\n", __func__);
	kfree(buf);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops adc122s101_ring_setup_ops = {
	.preenable = &adc122s101_ring_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
	.postdisable = &adc122s101_ring_postdisable,
};

int adc122s101_register_ring_funcs_and_init(struct iio_dev *indio_dev)
{
	int ret;

	printk("%s\n", __func__);

	indio_dev->buffer = iio_sw_rb_allocate(indio_dev);
	if (!indio_dev->buffer) {
		printk("%s: error allocating buffer\n", __func__);
		ret = -ENOMEM;
		goto error_ret;
	}
	/* Effectively select the ring buffer implementation */
	indio_dev->buffer->access = &ring_sw_access_funcs;
	indio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time,
						 &adc122s101_trigger_handler,
						 IRQF_ONESHOT,
						 indio_dev,
						 "adc122s101_consumer%d",
						 indio_dev->id);
	if (indio_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto error_deallocate_sw_rb;
	}

	/* Ring buffer functions - here trigger setup related */
	indio_dev->buffer->setup_ops = &adc122s101_ring_setup_ops;

	/* Flag that polled ring buffering is possible */
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	return 0;

error_deallocate_sw_rb:
	iio_sw_rb_free(indio_dev->buffer);
error_ret:
	return ret;
}

void adc122s101_ring_cleanup(struct iio_dev *indio_dev)
{
	printk("%s\n", __func__);

	iio_dealloc_pollfunc(indio_dev->pollfunc);
	iio_sw_rb_free(indio_dev->buffer);
}
