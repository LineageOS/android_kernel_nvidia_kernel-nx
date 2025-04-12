// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023 InvenSense, Inc.
 * Copyright (C) 2025 SylveonDeko
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>

#include "inv_icm42607x.h"
#include "inv_icm42607x_timestamp.h"
#include "inv_icm42607x_buffer.h"

/* FIFO header: 1 byte */
#define INV_ICM42607X_FIFO_HEADER_MSG		BIT(7)
#define INV_ICM42607X_FIFO_HEADER_ACCEL		BIT(6)
#define INV_ICM42607X_FIFO_HEADER_GYRO		BIT(5)
#define INV_ICM42607X_FIFO_HEADER_TMST_FSYNC	GENMASK(3, 2)
#define INV_ICM42607X_FIFO_HEADER_ODR_ACCEL	BIT(1)
#define INV_ICM42607X_FIFO_HEADER_ODR_GYRO	BIT(0)

struct inv_icm42607x_fifo_1sensor_packet {
	uint8_t header;
	struct inv_icm42607x_fifo_sensor_data data;
	int8_t temp;
} __packed;
#define INV_ICM42607X_FIFO_1SENSOR_PACKET_SIZE		8

struct inv_icm42607x_fifo_2sensors_packet {
	uint8_t header;
	struct inv_icm42607x_fifo_sensor_data accel;
	struct inv_icm42607x_fifo_sensor_data gyro;
	int8_t temp;
	__be16 timestamp;
} __packed;
#define INV_ICM42607X_FIFO_2SENSORS_PACKET_SIZE		16

/* Maximum FIFO watermark value (in bytes) */
#define INV_ICM42607X_FIFO_WATERMARK_MAX 2048

ssize_t inv_icm42607x_fifo_decode_packet(const void *packet, const void **accel,
										 const void **gyro, const int8_t **temp,
										 const void **timestamp, unsigned int *odr)
{
	const struct inv_icm42607x_fifo_1sensor_packet *pack1 = packet;
	const struct inv_icm42607x_fifo_2sensors_packet *pack2 = packet;
	uint8_t header = *((const uint8_t *)packet);

	/* FIFO empty */
	if (header & INV_ICM42607X_FIFO_HEADER_MSG) {
		*accel = NULL;
		*gyro = NULL;
		*temp = NULL;
		*timestamp = NULL;
		*odr = 0;
		return 0;
	}

	/* handle odr flags */
	*odr = 0;
	if (header & INV_ICM42607X_FIFO_HEADER_ODR_GYRO)
		*odr |= INV_ICM42607X_SENSOR_GYRO;
	if (header & INV_ICM42607X_FIFO_HEADER_ODR_ACCEL)
		*odr |= INV_ICM42607X_SENSOR_ACCEL;

	/* accel + gyro */
	if ((header & INV_ICM42607X_FIFO_HEADER_ACCEL) &&
		(header & INV_ICM42607X_FIFO_HEADER_GYRO)) {
		*accel = &pack2->accel;
	*gyro = &pack2->gyro;
	*temp = &pack2->temp;
	*timestamp = &pack2->timestamp;
	return INV_ICM42607X_FIFO_2SENSORS_PACKET_SIZE;
		}

		/* accel only */
		if (header & INV_ICM42607X_FIFO_HEADER_ACCEL) {
			*accel = &pack1->data;
			*gyro = NULL;
			*temp = &pack1->temp;
			*timestamp = NULL;
			return INV_ICM42607X_FIFO_1SENSOR_PACKET_SIZE;
		}

		/* gyro only */
		if (header & INV_ICM42607X_FIFO_HEADER_GYRO) {
			*accel = NULL;
			*gyro = &pack1->data;
			*temp = &pack1->temp;
			*timestamp = NULL;
			return INV_ICM42607X_FIFO_1SENSOR_PACKET_SIZE;
		}

		/* invalid packet if here */
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_fifo_decode_packet);

void inv_icm42607x_buffer_update_fifo_period(struct inv_icm42607x_state *st)
{
	uint32_t period_gyro, period_accel, period;

	if (st->fifo.en & INV_ICM42607X_SENSOR_GYRO)
		period_gyro = inv_icm42607x_odr_to_period(st->conf.gyro.odr);
	else
		period_gyro = U32_MAX;

	if (st->fifo.en & INV_ICM42607X_SENSOR_ACCEL)
		period_accel = inv_icm42607x_odr_to_period(st->conf.accel.odr);
	else
		period_accel = U32_MAX;

	if (period_gyro <= period_accel)
		period = period_gyro;
	else
		period = period_accel;

	st->fifo.period = period;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_buffer_update_fifo_period);

int inv_icm42607x_buffer_set_fifo_en(struct inv_icm42607x_state *st,
									 unsigned int fifo_en)
{
	unsigned int val;
	int ret;

	/* update FIFO EN bits for accel and gyro */
	val = 0;
	if (fifo_en & INV_ICM42607X_SENSOR_GYRO)
		val |= INV_ICM42607X_FIFO_CONFIG1_MODE;
	if (fifo_en & INV_ICM42607X_SENSOR_ACCEL)
		val |= INV_ICM42607X_FIFO_CONFIG1_MODE;
	if (fifo_en & INV_ICM42607X_SENSOR_TEMP)
		val |= INV_ICM42607X_FIFO_CONFIG1_MODE;

	ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG1, val);
	if (ret)
		return ret;

	st->fifo.en = fifo_en;
	inv_icm42607x_buffer_update_fifo_period(st);

	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_buffer_set_fifo_en);

static size_t inv_icm42607x_get_packet_size(unsigned int fifo_en)
{
	size_t packet_size;

	if ((fifo_en & INV_ICM42607X_SENSOR_GYRO) &&
		(fifo_en & INV_ICM42607X_SENSOR_ACCEL))
		packet_size = INV_ICM42607X_FIFO_2SENSORS_PACKET_SIZE;
	else
		packet_size = INV_ICM42607X_FIFO_1SENSOR_PACKET_SIZE;

	return packet_size;
}

static unsigned int inv_icm42607x_wm_truncate(unsigned int watermark,
											  size_t packet_size)
{
	size_t wm_size;
	unsigned int wm;

	wm_size = watermark * packet_size;
	if (wm_size > INV_ICM42607X_FIFO_WATERMARK_MAX)
		wm_size = INV_ICM42607X_FIFO_WATERMARK_MAX;

	wm = wm_size / packet_size;

	return wm;
}

/**
 * inv_icm42607x_buffer_update_watermark - update watermark FIFO threshold
 * @st:	driver internal state
 *
 * Returns 0 on success, a negative error code otherwise.
 */
int inv_icm42607x_buffer_update_watermark(struct inv_icm42607x_state *st)
{
	size_t packet_size, wm_size;
	unsigned int wm_gyro, wm_accel, watermark;
	uint32_t period_gyro, period_accel, period;
	uint32_t latency_gyro, latency_accel, latency;
	bool restore;
	uint8_t raw_wm[2];
	int ret;

	packet_size = inv_icm42607x_get_packet_size(st->fifo.en);

	/* compute sensors latency, depending on sensor watermark and odr */
	wm_gyro = inv_icm42607x_wm_truncate(st->fifo.watermark.gyro, packet_size);
	wm_accel = inv_icm42607x_wm_truncate(st->fifo.watermark.accel, packet_size);
	/* use us for odr to avoid overflow using 32 bits values */
	period_gyro = inv_icm42607x_odr_to_period(st->conf.gyro.odr) / 1000UL;
	period_accel = inv_icm42607x_odr_to_period(st->conf.accel.odr) / 1000UL;
	latency_gyro = period_gyro * wm_gyro;
	latency_accel = period_accel * wm_accel;

	/* 0 value for watermark means that the sensor is turned off */
	if (latency_gyro == 0) {
		watermark = wm_accel;
	} else if (latency_accel == 0) {
		watermark = wm_gyro;
	} else {
		/* compute the smallest latency that is a multiple of both */
		if (latency_gyro <= latency_accel)
			latency = latency_gyro - (latency_accel % latency_gyro);
		else
			latency = latency_accel - (latency_gyro % latency_accel);
		/* use the shortest period */
		if (period_gyro <= period_accel)
			period = period_gyro;
		else
			period = period_accel;
		/* all this works because periods are multiple of each others */
		watermark = latency / period;
		if (watermark < 1)
			watermark = 1;
	}

	/* compute watermark value in bytes */
	wm_size = watermark * packet_size;

	/* changing FIFO watermark requires to turn off watermark interrupt */
	ret = regmap_update_bits_check(st->map, INV_ICM42607X_REG_INT_SOURCE0,
								   INV_ICM42607X_INT_SOURCE0_FIFO_THS_INT1_EN,
								0, &restore);
	if (ret)
		return ret;

	/* Setup watermark */
	raw_wm[0] = wm_size & 0xFF;
	raw_wm[1] = (wm_size >> 8) & 0x0F;

	ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG2, raw_wm[0]);
	if (ret)
		return ret;

	ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG3,
					   INV_ICM42607X_FIFO_CONFIG3_WM_UPPER(raw_wm[1]));
	if (ret)
		return ret;

	/* restore watermark interrupt */
	if (restore) {
		ret = regmap_update_bits(st->map, INV_ICM42607X_REG_INT_SOURCE0,
								 INV_ICM42607X_INT_SOURCE0_FIFO_THS_INT1_EN,
						   INV_ICM42607X_INT_SOURCE0_FIFO_THS_INT1_EN);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_buffer_update_watermark);

static int inv_icm42607x_buffer_preenable(struct iio_dev *indio_dev)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	struct device *dev = regmap_get_device(st->map);

	pm_runtime_get_sync(dev);

	return 0;
}

/*
 * update_scan_mode callback is turning sensors on and setting data FIFO enable
 * bits.
 */
static int inv_icm42607x_buffer_postenable(struct iio_dev *indio_dev)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	/* exit if FIFO is already on */
	if (st->fifo.on) {
		ret = 0;
		goto out_on;
	}

	/* set FIFO threshold interrupt */
	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_INT_SOURCE0,
							 INV_ICM42607X_INT_SOURCE0_FIFO_THS_INT1_EN,
						  INV_ICM42607X_INT_SOURCE0_FIFO_THS_INT1_EN);
	if (ret)
		goto out_unlock;

	/* flush FIFO data */
	ret = regmap_write(st->map, INV_ICM42607X_REG_SIGNAL_PATH_RESET,
					   INV_ICM42607X_SIGNAL_PATH_RESET_FIFO_FLUSH);
	if (ret)
		goto out_unlock;

	/* set FIFO in streaming mode */
	ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG1,
					   INV_ICM42607X_FIFO_CONFIG1_MODE);
	if (ret)
		goto out_unlock;

	/* workaround: first read of FIFO count after reset is always 0 */
	ret = regmap_bulk_read(st->map, INV_ICM42607X_REG_FIFO_COUNTH, st->buffer, 2);
	if (ret)
		goto out_unlock;

	out_on:
	/* increase FIFO on counter */
	st->fifo.on++;
	out_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static int inv_icm42607x_buffer_predisable(struct iio_dev *indio_dev)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	/* exit if there are several sensors using the FIFO */
	if (st->fifo.on > 1) {
		ret = 0;
		goto out_off;
	}

	/* set FIFO in bypass mode */
	ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG1,
					   INV_ICM42607X_FIFO_CONFIG1_BYPASS);
	if (ret)
		goto out_unlock;

	/* flush FIFO data */
	ret = regmap_write(st->map, INV_ICM42607X_REG_SIGNAL_PATH_RESET,
					   INV_ICM42607X_SIGNAL_PATH_RESET_FIFO_FLUSH);
	if (ret)
		goto out_unlock;

	/* disable FIFO threshold interrupt */
	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_INT_SOURCE0,
							 INV_ICM42607X_INT_SOURCE0_FIFO_THS_INT1_EN, 0);
	if (ret)
		goto out_unlock;

	out_off:
	/* decrease FIFO on counter */
	st->fifo.on--;
	out_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static int inv_icm42607x_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	unsigned int sensor;
	unsigned int *watermark;
	struct inv_icm42607x_timestamp *ts;
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	unsigned int sleep_temp = 0;
	unsigned int sleep_sensor = 0;
	unsigned int sleep;
	int ret;

	if (indio_dev == st->indio_gyro) {
		sensor = INV_ICM42607X_SENSOR_GYRO;
		watermark = &st->fifo.watermark.gyro;
		ts = iio_priv(st->indio_gyro);
	} else if (indio_dev == st->indio_accel) {
		sensor = INV_ICM42607X_SENSOR_ACCEL;
		watermark = &st->fifo.watermark.accel;
		ts = iio_priv(st->indio_accel);
	} else {
		return -EINVAL;
	}

	mutex_lock(&st->lock);

	ret = inv_icm42607x_buffer_set_fifo_en(st, st->fifo.en & ~sensor);
	if (ret)
		goto out_unlock;

	*watermark = 0;
	ret = inv_icm42607x_buffer_update_watermark(st);
	if (ret)
		goto out_unlock;

	conf.mode = INV_ICM42607X_SENSOR_MODE_OFF;
	if (sensor == INV_ICM42607X_SENSOR_GYRO)
		ret = inv_icm42607x_set_gyro_conf(st, &conf, &sleep_sensor);
	else
		ret = inv_icm42607x_set_accel_conf(st, &conf, &sleep_sensor);
	if (ret)
		goto out_unlock;

	/* if FIFO is off, turn temperature off */
	if (!st->fifo.on)
		ret = inv_icm42607x_set_temp_conf(st, false, &sleep_temp);

	inv_icm42607x_timestamp_reset(ts);

	out_unlock:
	mutex_unlock(&st->lock);

	/* sleep maximum required time */
	if (sleep_sensor > sleep_temp)
		sleep = sleep_sensor;
	else
		sleep = sleep_temp;
	if (sleep)
		msleep(sleep);

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

const struct iio_buffer_setup_ops inv_icm42607x_buffer_ops = {
	.preenable = inv_icm42607x_buffer_preenable,
	.postenable = inv_icm42607x_buffer_postenable,
	.predisable = inv_icm42607x_buffer_predisable,
	.postdisable = inv_icm42607x_buffer_postdisable,
};
EXPORT_SYMBOL_GPL(inv_icm42607x_buffer_ops);

int inv_icm42607x_buffer_fifo_read(struct inv_icm42607x_state *st,
								   unsigned int max)
{
	size_t max_count;
	__be16 *raw_fifo_count;
	ssize_t i, size;
	const void *accel, *gyro, *timestamp;
	const int8_t *temp;
	unsigned int odr;
	int ret;

	/* reset all samples counters */
	st->fifo.count = 0;
	st->fifo.nb.gyro = 0;
	st->fifo.nb.accel = 0;
	st->fifo.nb.total = 0;

	/* compute maximum FIFO read size */
	if (max == 0)
		max_count = sizeof(st->fifo.data);
	else
		max_count = max * inv_icm42607x_get_packet_size(st->fifo.en);

	/* read FIFO count value */
	raw_fifo_count = (__be16 *)st->buffer;
	ret = regmap_bulk_read(st->map, INV_ICM42607X_REG_FIFO_COUNTH,
						   raw_fifo_count, sizeof(*raw_fifo_count));
	if (ret)
		return ret;
	st->fifo.count = be16_to_cpup(raw_fifo_count);

	/* check and clamp FIFO count value */
	if (st->fifo.count == 0)
		return 0;
	if (st->fifo.count > max_count)
		st->fifo.count = max_count;

	/* read all FIFO data in internal buffer */
	ret = regmap_bulk_read(st->map, INV_ICM42607X_REG_FIFO_DATA,
						   st->fifo.data, st->fifo.count);
	if (ret)
		return ret;

	/* compute number of samples for each sensor */
	for (i = 0; i < st->fifo.count; i += size) {
		size = inv_icm42607x_fifo_decode_packet(&st->fifo.data[i],
												&accel, &gyro, &temp, &timestamp, &odr);
		if (size <= 0)
			break;
		if (gyro != NULL && inv_icm42607x_fifo_is_data_valid(gyro))
			st->fifo.nb.gyro++;
		if (accel != NULL && inv_icm42607x_fifo_is_data_valid(accel))
			st->fifo.nb.accel++;
		st->fifo.nb.total++;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_buffer_fifo_read);

int inv_icm42607x_buffer_fifo_parse(struct inv_icm42607x_state *st)
{
	struct inv_icm42607x_timestamp *ts;
	int ret;

	if (st->fifo.nb.total == 0)
		return 0;

	/* handle gyroscope timestamp and FIFO data parsing */
	ts = iio_priv(st->indio_gyro);
	inv_icm42607x_timestamp_interrupt(ts, st->fifo.period, st->fifo.nb.total,
									  st->fifo.nb.gyro, st->timestamp.gyro);
	if (st->fifo.nb.gyro > 0) {
		ret = inv_icm42607x_gyro_parse_fifo(st->indio_gyro);
		if (ret)
			return ret;
	}

	/* handle accelerometer timestamp and FIFO data parsing */
	ts = iio_priv(st->indio_accel);
	inv_icm42607x_timestamp_interrupt(ts, st->fifo.period, st->fifo.nb.total,
									  st->fifo.nb.accel, st->timestamp.accel);
	if (st->fifo.nb.accel > 0) {
		ret = inv_icm42607x_accel_parse_fifo(st->indio_accel);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_buffer_fifo_parse);

int inv_icm42607x_buffer_hwfifo_flush(struct inv_icm42607x_state *st,
									  unsigned int count)
{
	struct inv_icm42607x_timestamp *ts;
	int64_t gyro_ts, accel_ts;
	int ret;

	gyro_ts = iio_get_time_ns(st->indio_gyro);
	accel_ts = iio_get_time_ns(st->indio_accel);

	ret = inv_icm42607x_buffer_fifo_read(st, count);
	if (ret)
		return ret;

	if (st->fifo.nb.total == 0)
		return 0;

	if (st->fifo.nb.gyro > 0) {
		ts = iio_priv(st->indio_gyro);
		inv_icm42607x_timestamp_interrupt(ts, st->fifo.period,
										  st->fifo.nb.total, st->fifo.nb.gyro,
									gyro_ts);
		ret = inv_icm42607x_gyro_parse_fifo(st->indio_gyro);
		if (ret)
			return ret;
	}

	if (st->fifo.nb.accel > 0) {
		ts = iio_priv(st->indio_accel);
		inv_icm42607x_timestamp_interrupt(ts, st->fifo.period,
										  st->fifo.nb.total, st->fifo.nb.accel,
									accel_ts);
		ret = inv_icm42607x_accel_parse_fifo(st->indio_accel);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_buffer_hwfifo_flush);

int inv_icm42607x_buffer_init(struct inv_icm42607x_state *st)
{
	unsigned int val;
	int ret;

	/* Configure FIFO_COUNT format in bytes and big endian */
	val = INV_ICM42607X_INTF_CONFIG0_FIFO_COUNT_ENDIAN;
	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG0,
							 val, val);
	if (ret)
		return ret;

	/* Initialize FIFO in bypass mode */
	return regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG1,
						INV_ICM42607X_FIFO_CONFIG1_BYPASS);
}
EXPORT_SYMBOL_GPL(inv_icm42607x_buffer_init);
