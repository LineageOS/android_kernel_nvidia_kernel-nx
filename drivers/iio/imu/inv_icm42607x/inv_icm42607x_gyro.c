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
#include <linux/math64.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>

#include "inv_icm42607x.h"
#include "inv_icm42607x_temp.h"
#include "inv_icm42607x_buffer.h"
#include "inv_icm42607x_timestamp.h"

#define INV_ICM42607X_GYRO_CHAN(_modifier, _index, _ext_info)		\
{								\
	.type = IIO_ANGL_VEL,					\
	.modified = 1,						\
	.channel2 = _modifier,					\
	.info_mask_separate =					\
	BIT(IIO_CHAN_INFO_RAW) |			\
	BIT(IIO_CHAN_INFO_CALIBBIAS),			\
	.info_mask_shared_by_type =				\
	BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all =				\
	BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_BE,				\
	},							\
	.ext_info = _ext_info,					\
}

enum inv_icm42607x_gyro_scan {
	INV_ICM42607X_GYRO_SCAN_X,
	INV_ICM42607X_GYRO_SCAN_Y,
	INV_ICM42607X_GYRO_SCAN_Z,
	INV_ICM42607X_GYRO_SCAN_TEMP,
	INV_ICM42607X_GYRO_SCAN_TIMESTAMP,
};

static const struct iio_chan_spec_ext_info inv_icm42607x_gyro_ext_infos[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_ALL, inv_icm42607x_get_mount_matrix),
	{},
};

static const struct iio_chan_spec inv_icm42607x_gyro_channels[] = {
	INV_ICM42607X_GYRO_CHAN(IIO_MOD_X, INV_ICM42607X_GYRO_SCAN_X,
							inv_icm42607x_gyro_ext_infos),
							INV_ICM42607X_GYRO_CHAN(IIO_MOD_Y, INV_ICM42607X_GYRO_SCAN_Y,
													inv_icm42607x_gyro_ext_infos),
													INV_ICM42607X_GYRO_CHAN(IIO_MOD_Z, INV_ICM42607X_GYRO_SCAN_Z,
																			inv_icm42607x_gyro_ext_infos),
																			INV_ICM42607X_TEMP_CHAN(INV_ICM42607X_GYRO_SCAN_TEMP),
																			IIO_CHAN_SOFT_TIMESTAMP(INV_ICM42607X_GYRO_SCAN_TIMESTAMP),
};

/*
 * IIO buffer data: size must be a power of 2 and timestamp aligned
 * 16 bytes: 6 bytes angular velocity, 2 bytes temperature, 8 bytes timestamp
 */
struct inv_icm42607x_gyro_buffer {
	struct inv_icm42607x_fifo_sensor_data gyro;
	int16_t temp;
	int64_t timestamp __aligned(8);
};

#define INV_ICM42607X_SCAN_MASK_GYRO_3AXIS				\
(BIT(INV_ICM42607X_GYRO_SCAN_X) |				\
BIT(INV_ICM42607X_GYRO_SCAN_Y) |				\
BIT(INV_ICM42607X_GYRO_SCAN_Z))

#define INV_ICM42607X_SCAN_MASK_TEMP	BIT(INV_ICM42607X_GYRO_SCAN_TEMP)

static const unsigned long inv_icm42607x_gyro_scan_masks[] = {
	/* 3-axis gyro + temperature */
	INV_ICM42607X_SCAN_MASK_GYRO_3AXIS | INV_ICM42607X_SCAN_MASK_TEMP,
	0,
};

/* enable gyroscope sensor and FIFO write */
static int inv_icm42607x_gyro_update_scan_mode(struct iio_dev *indio_dev,
											   const unsigned long *scan_mask)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	struct inv_icm42607x_timestamp *ts = iio_priv(indio_dev);
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	unsigned int fifo_en = 0;
	unsigned int sleep_gyro = 0;
	unsigned int sleep_temp = 0;
	unsigned int sleep;
	int ret;

	mutex_lock(&st->lock);

	if (*scan_mask & INV_ICM42607X_SCAN_MASK_TEMP) {
		/* enable temp sensor */
		ret = inv_icm42607x_set_temp_conf(st, true, &sleep_temp);
		if (ret)
			goto out_unlock;
		fifo_en |= INV_ICM42607X_SENSOR_TEMP;
	}

	if (*scan_mask & INV_ICM42607X_SCAN_MASK_GYRO_3AXIS) {
		/* enable gyro sensor */
		conf.mode = INV_ICM42607X_SENSOR_MODE_LOW_NOISE;
		ret = inv_icm42607x_set_gyro_conf(st, &conf, &sleep_gyro);
		if (ret)
			goto out_unlock;
		fifo_en |= INV_ICM42607X_SENSOR_GYRO;
	}

	/* update data FIFO write */
	inv_icm42607x_timestamp_apply_odr(ts, 0, 0, 0);
	ret = inv_icm42607x_buffer_set_fifo_en(st, fifo_en | st->fifo.en);
	if (ret)
		goto out_unlock;

	ret = inv_icm42607x_buffer_update_watermark(st);

	out_unlock:
	mutex_unlock(&st->lock);
	/* sleep maximum required time */
	if (sleep_gyro > sleep_temp)
		sleep = sleep_gyro;
	else
		sleep = sleep_temp;
	if (sleep)
		msleep(sleep);
	return ret;
}

static int inv_icm42607x_gyro_read_sensor(struct inv_icm42607x_state *st,
										  struct iio_chan_spec const *chan,
										  int16_t *val)
{
	struct device *dev = regmap_get_device(st->map);
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	unsigned int reg;
	__be16 *data;
	int ret;

	if (chan->type != IIO_ANGL_VEL)
		return -EINVAL;

	switch (chan->channel2) {
		case IIO_MOD_X:
			reg = INV_ICM42607X_REG_GYRO_DATA_X1;
			break;
		case IIO_MOD_Y:
			reg = INV_ICM42607X_REG_GYRO_DATA_Y1;
			break;
		case IIO_MOD_Z:
			reg = INV_ICM42607X_REG_GYRO_DATA_Z1;
			break;
		default:
			return -EINVAL;
	}

	pm_runtime_get_sync(dev);
	mutex_lock(&st->lock);

	/* enable gyro sensor */
	conf.mode = INV_ICM42607X_SENSOR_MODE_LOW_NOISE;
	ret = inv_icm42607x_set_gyro_conf(st, &conf, NULL);
	if (ret)
		goto exit;

	/* read gyro register data */
	data = (__be16 *)&st->buffer[0];
	ret = regmap_bulk_read(st->map, reg, data, sizeof(*data));
	if (ret)
		goto exit;

	*val = (int16_t)be16_to_cpup(data);
	if (*val == INV_ICM42607X_DATA_INVALID)
		ret = -EINVAL;
	exit:
	mutex_unlock(&st->lock);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);
	return ret;
}

/* IIO format int + nano */
static const int inv_icm42607x_gyro_scale[] = {
	/* +/- 2000dps => 0.001065264 rad/s */
	[2 * INV_ICM42607X_GYRO_FS_2000DPS] = 0,
	[2 * INV_ICM42607X_GYRO_FS_2000DPS + 1] = 1065264,
	/* +/- 1000dps => 0.000532632 rad/s */
	[2 * INV_ICM42607X_GYRO_FS_1000DPS] = 0,
	[2 * INV_ICM42607X_GYRO_FS_1000DPS + 1] = 532632,
	/* +/- 500dps => 0.000266316 rad/s */
	[2 * INV_ICM42607X_GYRO_FS_500DPS] = 0,
	[2 * INV_ICM42607X_GYRO_FS_500DPS + 1] = 266316,
	/* +/- 250dps => 0.000133158 rad/s */
	[2 * INV_ICM42607X_GYRO_FS_250DPS] = 0,
	[2 * INV_ICM42607X_GYRO_FS_250DPS + 1] = 133158,
};

static int inv_icm42607x_gyro_read_scale(struct inv_icm42607x_state *st,
										 int *val, int *val2)
{
	unsigned int idx;

	idx = st->conf.gyro.fs;

	*val = inv_icm42607x_gyro_scale[2 * idx];
	*val2 = inv_icm42607x_gyro_scale[2 * idx + 1];
	return IIO_VAL_INT_PLUS_NANO;
}

static int inv_icm42607x_gyro_write_scale(struct inv_icm42607x_state *st,
										  int val, int val2)
{
	struct device *dev = regmap_get_device(st->map);
	unsigned int idx;
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	int ret;

	for (idx = 0; idx < ARRAY_SIZE(inv_icm42607x_gyro_scale); idx += 2) {
		if (val == inv_icm42607x_gyro_scale[idx] &&
			val2 == inv_icm42607x_gyro_scale[idx + 1])
			break;
	}
	if (idx >= ARRAY_SIZE(inv_icm42607x_gyro_scale))
		return -EINVAL;

	conf.fs = idx / 2;

	pm_runtime_get_sync(dev);
	mutex_lock(&st->lock);

	ret = inv_icm42607x_set_gyro_conf(st, &conf, NULL);

	mutex_unlock(&st->lock);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

/* IIO format int + micro */
static const int inv_icm42607x_gyro_odr[] = {
	/* 12.5Hz */
	12, 500000,
	/* 25Hz */
	25, 0,
	/* 50Hz */
	50, 0,
	/* 100Hz */
	100, 0,
	/* 200Hz */
	200, 0,
	/* 400Hz */
	400, 0,
	/* 800Hz */
	800, 0,
	/* 1600Hz */
	1600, 0,
};

static const int inv_icm42607x_gyro_odr_conv[] = {
	INV_ICM42607X_ODR_12_5HZ,
	INV_ICM42607X_ODR_25HZ,
	INV_ICM42607X_ODR_50HZ,
	INV_ICM42607X_ODR_100HZ,
	INV_ICM42607X_ODR_200HZ,
	INV_ICM42607X_ODR_400HZ,
	INV_ICM42607X_ODR_800HZ,
	INV_ICM42607X_ODR_1600HZ,
};

static int inv_icm42607x_gyro_read_odr(struct inv_icm42607x_state *st,
									   int *val, int *val2)
{
	unsigned int odr;
	unsigned int i;

	odr = st->conf.gyro.odr;

	for (i = 0; i < ARRAY_SIZE(inv_icm42607x_gyro_odr_conv); ++i) {
		if (inv_icm42607x_gyro_odr_conv[i] == odr)
			break;
	}
	if (i >= ARRAY_SIZE(inv_icm42607x_gyro_odr_conv))
		return -EINVAL;

	*val = inv_icm42607x_gyro_odr[2 * i];
	*val2 = inv_icm42607x_gyro_odr[2 * i + 1];

	return IIO_VAL_INT_PLUS_MICRO;
}

static int inv_icm42607x_gyro_write_odr(struct iio_dev *indio_dev,
										int val, int val2)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	struct inv_icm42607x_timestamp *ts = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	unsigned int idx;
	struct inv_icm42607x_sensor_conf conf = INV_ICM42607X_SENSOR_CONF_INIT;
	int ret;

	for (idx = 0; idx < ARRAY_SIZE(inv_icm42607x_gyro_odr); idx += 2) {
		if (val == inv_icm42607x_gyro_odr[idx] &&
			val2 == inv_icm42607x_gyro_odr[idx + 1])
			break;
	}
	if (idx >= ARRAY_SIZE(inv_icm42607x_gyro_odr))
		return -EINVAL;

	conf.odr = inv_icm42607x_gyro_odr_conv[idx / 2];

	pm_runtime_get_sync(dev);
	mutex_lock(&st->lock);

	ret = inv_icm42607x_timestamp_update_odr(ts, inv_icm42607x_odr_to_period(conf.odr),
											 iio_buffer_enabled(indio_dev));
	if (ret)
		goto out_unlock;

	ret = inv_icm42607x_set_gyro_conf(st, &conf, NULL);
	if (ret)
		goto out_unlock;
	inv_icm42607x_buffer_update_fifo_period(st);
	inv_icm42607x_buffer_update_watermark(st);

	out_unlock:
	mutex_unlock(&st->lock);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	return ret;
}

/*
 * Calibration bias values, IIO range format int + nano.
 * Value is limited to +/-64dps coded on 12 bits signed. Step is 1/32 dps.
 */
static int inv_icm42607x_gyro_write_calibbias(struct inv_icm42607x_state *st,
											  struct iio_chan_spec const *chan,
											  int val, int val2)
{
	/* Not actually supported in the ICM-42607P registers */
	return -EOPNOTSUPP;
}

static int inv_icm42607x_gyro_read_calibbias(struct inv_icm42607x_state *st,
											 struct iio_chan_spec const *chan,
											 int *val, int *val2)
{
	/* Not actually supported in the ICM-42607P registers */
	return -EOPNOTSUPP;
}

static int inv_icm42607x_gyro_read_raw(struct iio_dev *indio_dev,
									   struct iio_chan_spec const *chan,
									   int *val, int *val2, long mask)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int16_t data;
	int ret;

	switch (chan->type) {
		case IIO_ANGL_VEL:
			break;
		case IIO_TEMP:
			return inv_icm42607x_temp_read_raw(indio_dev, chan, val, val2, mask);
		default:
			return -EINVAL;
	}

	switch (mask) {
		case IIO_CHAN_INFO_RAW:
			ret = iio_device_claim_direct_mode(indio_dev);
			if (ret)
				return ret;
		ret = inv_icm42607x_gyro_read_sensor(st, chan, &data);
		iio_device_release_direct_mode(indio_dev);
		if (ret)
			return ret;
		*val = data;
		return IIO_VAL_INT;
		case IIO_CHAN_INFO_SCALE:
			return inv_icm42607x_gyro_read_scale(st, val, val2);
		case IIO_CHAN_INFO_SAMP_FREQ:
			return inv_icm42607x_gyro_read_odr(st, val, val2);
		case IIO_CHAN_INFO_CALIBBIAS:
			return inv_icm42607x_gyro_read_calibbias(st, chan, val, val2);
		default:
			return -EINVAL;
	}
}

static int inv_icm42607x_gyro_write_raw(struct iio_dev *indio_dev,
										struct iio_chan_spec const *chan,
										int val, int val2, long mask)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	if (chan->type != IIO_ANGL_VEL)
		return -EINVAL;

	switch (mask) {
		case IIO_CHAN_INFO_SCALE:
			ret = iio_device_claim_direct_mode(indio_dev);
			if (ret)
				return ret;
		ret = inv_icm42607x_gyro_write_scale(st, val, val2);
		iio_device_release_direct_mode(indio_dev);
		return ret;
		case IIO_CHAN_INFO_SAMP_FREQ:
			return inv_icm42607x_gyro_write_odr(indio_dev, val, val2);
		case IIO_CHAN_INFO_CALIBBIAS:
			ret = iio_device_claim_direct_mode(indio_dev);
			if (ret)
				return ret;
		ret = inv_icm42607x_gyro_write_calibbias(st, chan, val, val2);
		iio_device_release_direct_mode(indio_dev);
		return ret;
		default:
			return -EINVAL;
	}
}

static int inv_icm42607x_gyro_write_raw_get_fmt(struct iio_dev *indio_dev,
												struct iio_chan_spec const *chan,
												long mask)
{
	if (chan->type != IIO_ANGL_VEL)
		return -EINVAL;

	switch (mask) {
		case IIO_CHAN_INFO_SCALE:
			return IIO_VAL_INT_PLUS_NANO;
		case IIO_CHAN_INFO_SAMP_FREQ:
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_CHAN_INFO_CALIBBIAS:
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return -EINVAL;
	}
}

static int inv_icm42607x_gyro_hwfifo_set_watermark(struct iio_dev *indio_dev,
												   unsigned int val)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	st->fifo.watermark.gyro = val;
	ret = inv_icm42607x_buffer_update_watermark(st);

	mutex_unlock(&st->lock);

	return ret;
}

static int inv_icm42607x_gyro_hwfifo_flush(struct iio_dev *indio_dev,
										   unsigned int count)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	if (count == 0)
		return 0;

	mutex_lock(&st->lock);

	ret = inv_icm42607x_buffer_hwfifo_flush(st, count);
	if (!ret)
		ret = st->fifo.nb.gyro;

	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_info inv_icm42607x_gyro_info = {
	.read_raw = inv_icm42607x_gyro_read_raw,
	.write_raw = inv_icm42607x_gyro_write_raw,
	.write_raw_get_fmt = inv_icm42607x_gyro_write_raw_get_fmt,
	.debugfs_reg_access = inv_icm42607x_debugfs_reg,
	.update_scan_mode = inv_icm42607x_gyro_update_scan_mode,
	.hwfifo_set_watermark = inv_icm42607x_gyro_hwfifo_set_watermark,
	.hwfifo_flush_to_buffer = inv_icm42607x_gyro_hwfifo_flush,
};

struct iio_dev *inv_icm42607x_gyro_init(struct inv_icm42607x_state *st)
{
	struct device *dev = regmap_get_device(st->map);
	const char *name;
	struct inv_icm42607x_timestamp *ts;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	int ret;

	name = devm_kasprintf(dev, GFP_KERNEL, "%s-gyro", st->name);
	if (!name)
		return ERR_PTR(-ENOMEM);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*ts));
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);

	buffer = devm_iio_kfifo_allocate(dev);
	if (!buffer)
		return ERR_PTR(-ENOMEM);

	ts = iio_priv(indio_dev);
	inv_icm42607x_timestamp_init(ts, inv_icm42607x_odr_to_period(st->conf.gyro.odr));

	iio_device_set_drvdata(indio_dev, st);
	indio_dev->name = name;
	indio_dev->info = &inv_icm42607x_gyro_info;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = inv_icm42607x_gyro_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm42607x_gyro_channels);
	indio_dev->available_scan_masks = inv_icm42607x_gyro_scan_masks;
	indio_dev->setup_ops = &inv_icm42607x_buffer_ops;

	iio_device_attach_buffer(indio_dev, buffer);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ERR_PTR(ret);

	return indio_dev;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_gyro_init);

int inv_icm42607x_gyro_parse_fifo(struct iio_dev *indio_dev)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	struct inv_icm42607x_timestamp *ts = iio_priv(indio_dev);
	ssize_t i, size;
	unsigned int no;
	const void *accel, *gyro, *timestamp;
	const int8_t *temp;
	unsigned int odr;
	int64_t ts_val;
	struct inv_icm42607x_gyro_buffer buffer;

	/* parse all fifo packets */
	for (i = 0, no = 0; i < st->fifo.count; i += size, ++no) {
		size = inv_icm42607x_fifo_decode_packet(&st->fifo.data[i],
												&accel, &gyro, &temp, &timestamp, &odr);
		/* quit if error or FIFO is empty */
		if (size <= 0)
			return size;

		/* skip packet if no gyro data or data is invalid */
		if (gyro == NULL || !inv_icm42607x_fifo_is_data_valid(gyro))
			continue;

		/* update odr */
		if (odr & INV_ICM42607X_SENSOR_GYRO) {
			inv_icm42607x_timestamp_apply_odr(ts, st->fifo.period,
											  st->fifo.nb.total, no);
		}

		/* buffer is copied to userspace, zeroing it to avoid any data leak */
		memset(&buffer, 0, sizeof(buffer));
		memcpy(&buffer.gyro, gyro, sizeof(buffer.gyro));
		/* convert 8 bits FIFO temperature in high resolution format */
		buffer.temp = temp ? (*temp * 64) : 0;
		ts_val = inv_icm42607x_timestamp_pop(ts);
		iio_push_to_buffers_with_timestamp(indio_dev, &buffer, ts_val);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_gyro_parse_fifo);
