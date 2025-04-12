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
#include <linux/iio/iio.h>

#include "inv_icm42607x.h"

#define INV_ICM42607X_TEMP_CHAN(_index)					\
{								\
    .type = IIO_TEMP,					\
    .info_mask_separate =					\
    BIT(IIO_CHAN_INFO_RAW) |			\
    BIT(IIO_CHAN_INFO_OFFSET) |			\
    BIT(IIO_CHAN_INFO_SCALE),			\
    .scan_index = _index,					\
    .scan_type = {						\
        .sign = 's',					\
        .realbits = 16,					\
        .storagebits = 16,				\
    },							\
}

static int inv_icm42607x_temp_read(struct inv_icm42607x_state *st, int16_t *temp)
{
    struct device *dev = regmap_get_device(st->map);
    __be16 *raw;
    int ret;

    pm_runtime_get_sync(dev);
    mutex_lock(&st->lock);

    ret = inv_icm42607x_set_temp_conf(st, true, NULL);
    if (ret)
        goto exit;

    raw = (__be16 *)&st->buffer[0];
    ret = regmap_bulk_read(st->map, INV_ICM42607X_REG_TEMP_DATA1, raw, sizeof(*raw));
    if (ret)
        goto exit;

    *temp = (int16_t)be16_to_cpup(raw);
    if (*temp == INV_ICM42607X_DATA_INVALID)
        ret = -EINVAL;

    exit:
    mutex_unlock(&st->lock);
    pm_runtime_mark_last_busy(dev);
    pm_runtime_put_autosuspend(dev);

    return ret;
}

int inv_icm42607x_temp_read_raw(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *chan,
                                int *val, int *val2, long mask)
{
    struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
    int16_t temp;
    int ret;

    if (chan->type != IIO_TEMP)
        return -EINVAL;

    switch (mask) {
        case IIO_CHAN_INFO_RAW:
            ret = iio_device_claim_direct_mode(indio_dev);
            if (ret)
                return ret;
        ret = inv_icm42607x_temp_read(st, &temp);
        iio_device_release_direct_mode(indio_dev);
        if (ret)
            return ret;
        *val = temp;
        return IIO_VAL_INT;
        /*
         * T°C = (temp / 128) + 25
         * Tm°C = 1000 * ((temp * 100 / 12800) + 25)
         * scale: 100000 / 12800 ~= 7.8125
         * offset: 25000
         */
        case IIO_CHAN_INFO_SCALE:
            *val = 7;
            *val2 = 812500;
            return IIO_VAL_INT_PLUS_MICRO;
        case IIO_CHAN_INFO_OFFSET:
            *val = 25000;
            return IIO_VAL_INT;
        default:
            return -EINVAL;
    }
}
EXPORT_SYMBOL_GPL(inv_icm42607x_temp_read_raw);
