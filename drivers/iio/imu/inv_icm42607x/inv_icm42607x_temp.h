/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2023 InvenSense, Inc.
 * Copyright (C) 2025 SylveonDeko
 */

#ifndef INV_ICM42607X_TEMP_H_
#define INV_ICM42607X_TEMP_H_

#include <linux/iio/iio.h>

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

int inv_icm42607x_temp_read_raw(struct iio_dev *indio_dev,
                                struct iio_chan_spec const *chan,
                                int *val, int *val2, long mask);

#endif
