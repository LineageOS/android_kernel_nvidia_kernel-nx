// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023 InvenSense, Inc.
 * Copyright (C) 2025 SylveonDeko
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>

#include "inv_icm42607x.h"
#include "inv_icm42607x_buffer.h"
#include "inv_icm42607x_timestamp.h"

static const struct regmap_range_cfg inv_icm42607x_regmap_ranges[] = {
	{
		.name = "user bank",
		.range_min = 0x0000,
		.range_max = 0x00FF,
		.selector_reg = 0, /* not used */
		.selector_mask = 0, /* not used */
		.selector_shift = 0, /* not used */
		.window_start = 0,
		.window_len = 0x0100,
	},
};

const struct regmap_config inv_icm42607x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x00FF,
	.ranges = inv_icm42607x_regmap_ranges,
	.num_ranges = ARRAY_SIZE(inv_icm42607x_regmap_ranges),
	.cache_type = REGCACHE_NONE,
};
EXPORT_SYMBOL_GPL(inv_icm42607x_regmap_config);

struct inv_icm42607x_hw {
	uint8_t whoami;
	const char *name;
	const struct inv_icm42607x_conf *conf;
};

/* chip initial default configuration */
static const struct inv_icm42607x_conf inv_icm42607x_default_conf = {
	.gyro = {
		.mode = INV_ICM42607X_SENSOR_MODE_OFF,
		.fs = INV_ICM42607X_GYRO_FS_1000DPS,
		.odr = INV_ICM42607X_ODR_100HZ,
		.filter = INV_ICM42607X_FILTER_BW_25HZ,
	},
	.accel = {
		.mode = INV_ICM42607X_SENSOR_MODE_OFF,
		.fs = INV_ICM42607X_ACCEL_FS_4G,
		.odr = INV_ICM42607X_ODR_100HZ,
		.filter = INV_ICM42607X_FILTER_BW_25HZ,
	},
	.temp_en = false,
};

static const struct inv_icm42607x_hw inv_icm42607x_hw[INV_CHIP_NB] = {
	[INV_CHIP_ICM42607P] = {
		.whoami = INV_ICM42607P_WHOAMI,
		.name = "icm42607p",
		.conf = &inv_icm42607x_default_conf,
	},
	[INV_CHIP_ICM42607] = {
		.whoami = INV_ICM42607_WHOAMI,
		.name = "icm42670",
		.conf = &inv_icm42607x_default_conf,
	},
};

const struct iio_mount_matrix *
inv_icm42607x_get_mount_matrix(struct iio_dev *indio_dev,
							   const struct iio_chan_spec *chan)
{
	const struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);

	return &st->orientation;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_get_mount_matrix);

uint32_t inv_icm42607x_odr_to_period(enum inv_icm42607x_odr odr)
{
	static uint32_t odr_periods[INV_ICM42607X_ODR_NB] = {
		625000,
		1250000,
		2500000,
		5000000,
		10000000,
		20000000,
		40000000,
		80000000,
		160000000,
		320000000,
		640000000,
	};

	return odr_periods[odr];
}
EXPORT_SYMBOL_GPL(inv_icm42607x_odr_to_period);

static int inv_icm42607x_set_pwr_mgmt0(struct inv_icm42607x_state *st,
									   enum inv_icm42607x_sensor_mode gyro,
									   enum inv_icm42607x_sensor_mode accel,
									   bool temp, unsigned int *sleep_ms)
{
	enum inv_icm42607x_sensor_mode oldgyro = st->conf.gyro.mode;
	enum inv_icm42607x_sensor_mode oldaccel = st->conf.accel.mode;
	bool oldtemp = st->conf.temp_en;
	unsigned int sleepval;
	unsigned int val;
	int ret;

	if (gyro == oldgyro && accel == oldaccel && temp == oldtemp)
		return 0;

	val = INV_ICM42607X_PWR_MGMT0_GYRO(gyro) |
	INV_ICM42607X_PWR_MGMT0_ACCEL(accel);
	if (!temp)
		val |= INV_ICM42607X_PWR_MGMT0_ACCEL_LP_CLK_SEL;
	ret = regmap_write(st->map, INV_ICM42607X_REG_PWR_MGMT0, val);
	if (ret)
		return ret;

	st->conf.gyro.mode = gyro;
	st->conf.accel.mode = accel;
	st->conf.temp_en = temp;

	sleepval = 0;
	if (temp && !oldtemp) {
		if (sleepval < INV_ICM42607X_TEMP_STARTUP_TIME_MS)
			sleepval = INV_ICM42607X_TEMP_STARTUP_TIME_MS;
	}
	if (accel != oldaccel && oldaccel == INV_ICM42607X_SENSOR_MODE_OFF) {
		usleep_range(200, 300);
		if (sleepval < INV_ICM42607X_ACCEL_STARTUP_TIME_MS)
			sleepval = INV_ICM42607X_ACCEL_STARTUP_TIME_MS;
	}
	if (gyro != oldgyro) {
		if (oldgyro == INV_ICM42607X_SENSOR_MODE_OFF) {
			usleep_range(200, 300);
			if (sleepval < INV_ICM42607X_GYRO_STARTUP_TIME_MS)
				sleepval = INV_ICM42607X_GYRO_STARTUP_TIME_MS;
		} else if (gyro == INV_ICM42607X_SENSOR_MODE_OFF) {
			if (sleepval < INV_ICM42607X_GYRO_STOP_TIME_MS)
				sleepval = INV_ICM42607X_GYRO_STOP_TIME_MS;
		}
	}

	if (sleep_ms)
		*sleep_ms = sleepval;
	else if (sleepval)
		msleep(sleepval);

	return 0;
}

int inv_icm42607x_set_accel_conf(struct inv_icm42607x_state *st,
								 struct inv_icm42607x_sensor_conf *conf,
								 unsigned int *sleep_ms)
{
	struct inv_icm42607x_sensor_conf *oldconf = &st->conf.accel;
	unsigned int val;
	int ret;

	if (conf->mode < 0)
		conf->mode = oldconf->mode;
	if (conf->fs < 0)
		conf->fs = oldconf->fs;
	if (conf->odr < 0)
		conf->odr = oldconf->odr;
	if (conf->filter < 0)
		conf->filter = oldconf->filter;

	if (conf->fs != oldconf->fs || conf->odr != oldconf->odr) {
		val = INV_ICM42607X_ACCEL_CONFIG0_FS_SEL(conf->fs) |
		INV_ICM42607X_ACCEL_CONFIG0_ODR(conf->odr);
		ret = regmap_write(st->map, INV_ICM42607X_REG_ACCEL_CONFIG0, val);
		if (ret)
			return ret;
		oldconf->fs = conf->fs;
		oldconf->odr = conf->odr;
	}

	if (conf->filter != oldconf->filter) {
		if (conf->mode == INV_ICM42607X_SENSOR_MODE_LOW_POWER) {
			val = INV_ICM42607X_ACCEL_CONFIG1_AVG(conf->filter);
			ret = regmap_update_bits(st->map, INV_ICM42607X_REG_ACCEL_CONFIG1,
									 INV_ICM42607X_ACCEL_CONFIG1_AVG_MASK, val);
		} else {
			val = INV_ICM42607X_ACCEL_CONFIG1_FILTER(conf->filter);
			ret = regmap_update_bits(st->map, INV_ICM42607X_REG_ACCEL_CONFIG1,
									 INV_ICM42607X_ACCEL_CONFIG1_FILTER_MASK, val);
		}
		if (ret)
			return ret;
		oldconf->filter = conf->filter;
	}

	return inv_icm42607x_set_pwr_mgmt0(st, st->conf.gyro.mode, conf->mode,
									   st->conf.temp_en, sleep_ms);
}
EXPORT_SYMBOL_GPL(inv_icm42607x_set_accel_conf);

int inv_icm42607x_set_gyro_conf(struct inv_icm42607x_state *st,
								struct inv_icm42607x_sensor_conf *conf,
								unsigned int *sleep_ms)
{
	struct inv_icm42607x_sensor_conf *oldconf = &st->conf.gyro;
	unsigned int val;
	int ret;

	if (conf->mode < 0)
		conf->mode = oldconf->mode;
	if (conf->fs < 0)
		conf->fs = oldconf->fs;
	if (conf->odr < 0)
		conf->odr = oldconf->odr;
	if (conf->filter < 0)
		conf->filter = oldconf->filter;

	if (conf->fs != oldconf->fs || conf->odr != oldconf->odr) {
		val = INV_ICM42607X_GYRO_CONFIG0_FS_SEL(conf->fs) |
		INV_ICM42607X_GYRO_CONFIG0_ODR(conf->odr);
		ret = regmap_write(st->map, INV_ICM42607X_REG_GYRO_CONFIG0, val);
		if (ret)
			return ret;
		oldconf->fs = conf->fs;
		oldconf->odr = conf->odr;
	}

	if (conf->filter != oldconf->filter) {
		val = INV_ICM42607X_GYRO_CONFIG1_FILTER(conf->filter);
		ret = regmap_update_bits(st->map, INV_ICM42607X_REG_GYRO_CONFIG1,
								 INV_ICM42607X_GYRO_CONFIG1_FILTER_MASK, val);
		if (ret)
			return ret;
		oldconf->filter = conf->filter;
	}

	return inv_icm42607x_set_pwr_mgmt0(st, conf->mode, st->conf.accel.mode,
									   st->conf.temp_en, sleep_ms);
}
EXPORT_SYMBOL_GPL(inv_icm42607x_set_gyro_conf);

int inv_icm42607x_set_temp_conf(struct inv_icm42607x_state *st, bool enable,
								unsigned int *sleep_ms)
{
	unsigned int val;
	int ret;

	val = INV_ICM42607X_TEMP_CONFIG0_FILTER(INV_ICM42607X_FILTER_BW_34HZ);
	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_TEMP_CONFIG0,
							 INV_ICM42607X_TEMP_CONFIG0_FILTER_MASK, val);
	if (ret)
		return ret;

	return inv_icm42607x_set_pwr_mgmt0(st, st->conf.gyro.mode,
									   st->conf.accel.mode, enable,
									sleep_ms);
}
EXPORT_SYMBOL_GPL(inv_icm42607x_set_temp_conf);

int inv_icm42607x_debugfs_reg(struct iio_dev *indio_dev, unsigned int reg,
							  unsigned int writeval, unsigned int *readval)
{
	struct inv_icm42607x_state *st = iio_device_get_drvdata(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	if (readval)
		ret = regmap_read(st->map, reg, readval);
	else
		ret = regmap_write(st->map, reg, writeval);

	mutex_unlock(&st->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(inv_icm42607x_debugfs_reg);

static int inv_icm42607x_set_conf(struct inv_icm42607x_state *st,
								  const struct inv_icm42607x_conf *conf)
{
	unsigned int val;
	int ret;

	val = INV_ICM42607X_PWR_MGMT0_GYRO(conf->gyro.mode) |
	INV_ICM42607X_PWR_MGMT0_ACCEL(conf->accel.mode);
	if (!conf->temp_en)
		val |= INV_ICM42607X_PWR_MGMT0_ACCEL_LP_CLK_SEL;
	ret = regmap_write(st->map, INV_ICM42607X_REG_PWR_MGMT0, val);
	if (ret)
		return ret;

	val = INV_ICM42607X_GYRO_CONFIG0_FS_SEL(conf->gyro.fs) |
	INV_ICM42607X_GYRO_CONFIG0_ODR(conf->gyro.odr);
	ret = regmap_write(st->map, INV_ICM42607X_REG_GYRO_CONFIG0, val);
	if (ret)
		return ret;

	val = INV_ICM42607X_ACCEL_CONFIG0_FS_SEL(conf->accel.fs) |
	INV_ICM42607X_ACCEL_CONFIG0_ODR(conf->accel.odr);
	ret = regmap_write(st->map, INV_ICM42607X_REG_ACCEL_CONFIG0, val);
	if (ret)
		return ret;

	val = INV_ICM42607X_GYRO_CONFIG1_FILTER(conf->gyro.filter);
	ret = regmap_write(st->map, INV_ICM42607X_REG_GYRO_CONFIG1, val);
	if (ret)
		return ret;

	val = INV_ICM42607X_ACCEL_CONFIG1_FILTER(conf->accel.filter);
	ret = regmap_write(st->map, INV_ICM42607X_REG_ACCEL_CONFIG1, val);
	if (ret)
		return ret;

	st->conf = *conf;

	return 0;
}

/**
 *  inv_icm42607x_setup() - check and setup chip
 *  @st:	driver internal state
 *  @bus_setup:	callback for setting up bus specific registers
 *
 *  Returns 0 on success, a negative error code otherwise.
 */
static int inv_icm42607x_setup(struct inv_icm42607x_state *st,
							   inv_icm42607x_bus_setup bus_setup)
{
	const struct inv_icm42607x_hw *hw = &inv_icm42607x_hw[st->chip];
	const struct device *dev = regmap_get_device(st->map);
	unsigned int val;
	int ret;

	ret = regmap_read(st->map, INV_ICM42607X_REG_WHOAMI, &val);
	if (ret)
		return ret;

	if (val != hw->whoami) {
		dev_err(dev, "invalid whoami %#02x expected %#02x (%s)\n",
				val, hw->whoami, hw->name);
		return -ENODEV;
	}
	st->name = hw->name;

	ret = regmap_write(st->map, INV_ICM42607X_REG_SIGNAL_PATH_RESET,
					   INV_ICM42607X_SIGNAL_PATH_RESET_SOFT_RESET);
	if (ret)
		return ret;
	msleep(INV_ICM42607X_RESET_TIME_MS);

	ret = regmap_read(st->map, INV_ICM42607X_REG_INT_STATUS, &val);
	if (ret)
		return ret;
	if (!(val & INV_ICM42607X_INT_STATUS_RESET_DONE)) {
		dev_err(dev, "reset error, reset done bit not set\n");
		return -ENODEV;
	}

	ret = bus_setup(st);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG0,
							 INV_ICM42607X_INTF_CONFIG0_SENSOR_DATA_ENDIAN,
						  INV_ICM42607X_INTF_CONFIG0_SENSOR_DATA_ENDIAN);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG1,
							 INV_ICM42607X_INTF_CONFIG1_CLKSEL_MASK,
						  INV_ICM42607X_INTF_CONFIG1_CLKSEL_PLL);
	if (ret)
		return ret;

	return inv_icm42607x_set_conf(st, hw->conf);
}

static irqreturn_t inv_icm42607x_irq_timestamp(int irq, void *_data)
{
	struct inv_icm42607x_state *st = _data;

	st->timestamp.gyro = iio_get_time_ns(st->indio_gyro);
	st->timestamp.accel = iio_get_time_ns(st->indio_accel);

	return IRQ_WAKE_THREAD;
}

static irqreturn_t inv_icm42607x_irq_handler(int irq, void *_data)
{
	struct inv_icm42607x_state *st = _data;
	struct device *dev = regmap_get_device(st->map);
	unsigned int status;
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_read(st->map, INV_ICM42607X_REG_INT_STATUS, &status);
	if (ret)
		goto out_unlock;

	if (status & INV_ICM42607X_INT_STATUS_FIFO_FULL)
		dev_warn(dev, "FIFO full data lost!\n");

	if (status & INV_ICM42607X_INT_STATUS_FIFO_THS) {
		ret = inv_icm42607x_buffer_fifo_read(st, 0);
		if (ret) {
			dev_err(dev, "FIFO read error %d\n", ret);
			goto out_unlock;
		}
		ret = inv_icm42607x_buffer_fifo_parse(st);
		if (ret)
			dev_err(dev, "FIFO parsing error %d\n", ret);
	}

	out_unlock:
	mutex_unlock(&st->lock);
	return IRQ_HANDLED;
}

/**
 * inv_icm42607x_irq_init() - initialize int pin and interrupt handler
 * @st:		driver internal state
 * @irq:	irq number
 * @irq_type:	irq trigger type
 * @open_drain:	true if irq is open drain, false for push-pull
 * @irq_int2:	use INT2 pin instead of INT1
 *
 * Returns 0 on success, a negative error code otherwise.
 */
static int inv_icm42607x_irq_init(struct inv_icm42607x_state *st, int irq,
								  int irq_type, bool open_drain, bool irq_int2)
{
	struct device *dev = regmap_get_device(st->map);
	unsigned int val = 0; // Initialize val
	int ret;

	if (!irq_int2) {
		switch (irq_type & IRQ_TYPE_SENSE_MASK) {
			case IRQ_TYPE_EDGE_RISING:
			case IRQ_TYPE_LEVEL_HIGH:
				val = INV_ICM42607X_INT_CONFIG_INT1_ACTIVE_HIGH;
				break;
			default: // Defaults to FALLING or LOW
				val = INV_ICM42607X_INT_CONFIG_INT1_ACTIVE_LOW;
				break;
		}

		switch (irq_type & IRQ_TYPE_SENSE_MASK) {
			case IRQ_TYPE_LEVEL_LOW:
			case IRQ_TYPE_LEVEL_HIGH:
				val |= INV_ICM42607X_INT_CONFIG_INT1_LATCHED;
				break;
			default: // Edge triggered
				break;
		}

		if (!open_drain)
			val |= INV_ICM42607X_INT_CONFIG_INT1_PUSH_PULL;
	} else {
		switch (irq_type & IRQ_TYPE_SENSE_MASK) {
			case IRQ_TYPE_EDGE_RISING:
			case IRQ_TYPE_LEVEL_HIGH:
				val = INV_ICM42607X_INT_CONFIG_INT2_ACTIVE_HIGH;
				break;
			default: // Defaults to FALLING or LOW
				val = INV_ICM42607X_INT_CONFIG_INT2_ACTIVE_LOW;
				break;
		}

		switch (irq_type & IRQ_TYPE_SENSE_MASK) {
			case IRQ_TYPE_LEVEL_LOW:
			case IRQ_TYPE_LEVEL_HIGH:
				val |= INV_ICM42607X_INT_CONFIG_INT2_LATCHED;
				break;
			default: // Edge triggered
				break;
		}

		if (!open_drain)
			val |= INV_ICM42607X_INT_CONFIG_INT2_PUSH_PULL;
	}

	ret = regmap_write(st->map, INV_ICM42607X_REG_INT_CONFIG, val);
	if (ret)
		return ret;

	return devm_request_threaded_irq(dev, irq, inv_icm42607x_irq_timestamp,
									 inv_icm42607x_irq_handler, irq_type | IRQF_ONESHOT,
								  st->name, st);
}


static int inv_icm42607x_enable_regulator_vddio(struct inv_icm42607x_state *st)
{
	int ret;

	ret = regulator_enable(st->vddio_supply);
	if (ret)
		return ret;

	usleep_range(3000, 4000);

	return 0;
}

static void inv_icm42607x_disable_vdd_reg(void *_data)
{
	struct inv_icm42607x_state *st = _data;
	const struct device *dev = regmap_get_device(st->map);
	int ret;

	ret = regulator_disable(st->vdd_supply);
	if (ret)
		dev_err(dev, "failed to disable vdd error %d\n", ret);
}

static void inv_icm42607x_disable_vddio_reg(void *_data)
{
	struct inv_icm42607x_state *st = _data;
	const struct device *dev = regmap_get_device(st->map);
	int ret;

	ret = regulator_disable(st->vddio_supply);
	if (ret)
		dev_err(dev, "failed to disable vddio error %d\n", ret);
}

static void inv_icm42607x_disable_pm(void *_data)
{
	struct device *dev = _data;

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);
}

int inv_icm42607x_core_probe(struct regmap *regmap, int chip, int irq,
							 inv_icm42607x_bus_setup bus_setup)
{
	struct device *dev = regmap_get_device(regmap);
	struct inv_icm42607x_state *st;
	struct irq_data *irq_desc;
	int irq_type;
	bool open_drain;
	bool irq_int2;
	int ret;

	if (chip < 0 || chip >= INV_CHIP_NB) {
		dev_err(dev, "invalid chip = %d\n", chip);
		return -ENODEV;
	}

	irq_desc = irq_get_irq_data(irq);
	if (!irq_desc) {
		dev_err(dev, "could not find IRQ %d\n", irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(irq_desc);
	if (irq_type == IRQ_TYPE_NONE) // Check for NONE explicitly
		irq_type = IRQF_TRIGGER_FALLING;

	open_drain = device_property_read_bool(dev, "drive-open-drain");
	irq_int2 = device_property_read_bool(dev, "irq-int2");

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	dev_set_drvdata(dev, st);
	mutex_init(&st->lock);
	st->chip = chip;
	st->map = regmap;

	/* Read mount matrix */
	ret = of_iio_read_mount_matrix(dev, "invensense,mount-matrix",
								   &st->orientation);
	if (ret && ret != -ENOENT) {
		dev_err(dev, "failed to retrieve mounting matrix %d\n", ret);
		return ret;
	} else {
		/* If -ENOENT or success, clear ret for subsequent checks */
		ret = 0;
	}

	/* Get VDD regulator */
	st->vdd_supply = devm_regulator_get(dev, "vdd");
	if (IS_ERR(st->vdd_supply)) {
		long err_code = PTR_ERR(st->vdd_supply);
		if (err_code == -ENODEV) {
			dev_dbg(dev, "vdd regulator not found\n");
			/* Treat missing regulator as non-fatal, clear pointer */
			st->vdd_supply = NULL;
		} else {
			dev_err(dev, "Failed to get VDD regulator: %ld\n", err_code);
			return err_code; /* Return actual error code */
		}
	}

	/* Get VDDIO regulator */
	st->vddio_supply = devm_regulator_get(dev, "vddio");
	if (IS_ERR(st->vddio_supply)) {
		long err_code = PTR_ERR(st->vddio_supply);
		if (err_code == -ENODEV) {
			dev_dbg(dev, "vddio regulator not found\n");
			/* Treat missing regulator as non-fatal, clear pointer */
			st->vddio_supply = NULL;
		} else {
			dev_err(dev, "Failed to get VDDIO regulator: %ld\n", err_code);
			return err_code; /* Return actual error code */
		}
	}

	/* Enable VDD regulator if found */
	if (st->vdd_supply) {
		ret = regulator_enable(st->vdd_supply);
		if (ret) {
			dev_err(dev, "Failed to enable VDD regulator: %d\n", ret);
			return ret; /* Return actual error code */
		}
		msleep(INV_ICM42607X_POWER_UP_TIME_MS);

		ret = devm_add_action_or_reset(dev, inv_icm42607x_disable_vdd_reg, st);
		if (ret)
			return ret;
	}

	/* Enable VDDIO regulator if found */
	if (st->vddio_supply) {
		ret = inv_icm42607x_enable_regulator_vddio(st);
		if (ret) {
			dev_err(dev, "Failed to enable VDDIO regulator: %d\n", ret);
			return ret; /* Return actual error code */
		}

		ret = devm_add_action_or_reset(dev, inv_icm42607x_disable_vddio_reg, st);
		if (ret)
			return ret;
	}

	/* Setup chip registers (includes WHOAMI check, reset check, bus setup) */
	ret = inv_icm42607x_setup(st, bus_setup);
	if (ret)
		return ret; /* Return error from setup (e.g., WHOAMI fail) */

	ret = inv_icm42607x_timestamp_setup(st);
	if (ret)
		return ret;

	/* Initialize buffer/FIFO handling */
	ret = inv_icm42607x_buffer_init(st);
	if (ret)
		return ret;

	/* Initialize IIO device for Gyro */
	st->indio_gyro = inv_icm42607x_gyro_init(st);
	if (IS_ERR(st->indio_gyro))
		return PTR_ERR(st->indio_gyro);

	/* Initialize IIO device for Accel */
	st->indio_accel = inv_icm42607x_accel_init(st);
	if (IS_ERR(st->indio_accel))
		return PTR_ERR(st->indio_accel);

	/* Initialize interrupt handling */
	ret = inv_icm42607x_irq_init(st, irq, irq_type, open_drain, irq_int2);
	if (ret)
		return ret;

	/* Setup runtime power management */
	ret = pm_runtime_set_active(dev);
	if (ret)
		return ret;
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, INV_ICM42607X_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_put(dev); /* Initial decrement after enabling */

	/* Add cleanup action for PM runtime disable */
	return devm_add_action_or_reset(dev, inv_icm42607x_disable_pm, dev);
}
EXPORT_SYMBOL_GPL(inv_icm42607x_core_probe);

/*
 * Suspend saves sensors state and turns everything off.
 * Check first if runtime suspend has not already done the job.
 */
static int __maybe_unused inv_icm42607x_suspend(struct device *dev)
{
	struct inv_icm42607x_state *st = dev_get_drvdata(dev);
	int ret = 0;

	if (!st) // Check if driver data exists
		return 0;

	mutex_lock(&st->lock);

	st->suspended.gyro = st->conf.gyro.mode;
	st->suspended.accel = st->conf.accel.mode;
	st->suspended.temp = st->conf.temp_en;
	if (pm_runtime_suspended(dev)) {
		goto out_unlock;
	}

	if (st->fifo.on) {
		ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG1,
						   INV_ICM42607X_FIFO_CONFIG1_BYPASS);
		if (ret)
			goto out_unlock;
	}

	ret = inv_icm42607x_set_pwr_mgmt0(st, INV_ICM42607X_SENSOR_MODE_OFF,
									  INV_ICM42607X_SENSOR_MODE_OFF, false,
								   NULL);
	if (ret)
		goto out_unlock;

	if (!IS_ERR(st->vddio_supply))
		regulator_disable(st->vddio_supply);

	out_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

/*
 * System resume gets the system back on and restores the sensors state.
 * Manually put runtime power management in system active state.
 */
static int __maybe_unused inv_icm42607x_resume(struct device *dev)
{
	struct inv_icm42607x_state *st = dev_get_drvdata(dev);
	int ret = 0;

	if (!st) // Check if driver data exists
		return 0;

	mutex_lock(&st->lock);

	if (!IS_ERR(st->vddio_supply)) {
		ret = inv_icm42607x_enable_regulator_vddio(st);
		if (ret)
			goto out_unlock;
	}

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = inv_icm42607x_set_pwr_mgmt0(st, st->suspended.gyro,
									  st->suspended.accel,
								   st->suspended.temp, NULL);
	if (ret)
		goto out_unlock;

	if (st->fifo.on) {
		ret = regmap_write(st->map, INV_ICM42607X_REG_FIFO_CONFIG1,
						   INV_ICM42607X_FIFO_CONFIG1_MODE);
	}

	out_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

/* Runtime suspend will turn off sensors that are enabled by iio devices. */
static int __maybe_unused inv_icm42607x_runtime_suspend(struct device *dev)
{
	struct inv_icm42607x_state *st = dev_get_drvdata(dev);
	int ret = 0;

	if (!st) // Check if driver data exists
		return 0;

	mutex_lock(&st->lock);

	ret = inv_icm42607x_set_pwr_mgmt0(st, INV_ICM42607X_SENSOR_MODE_OFF,
									  INV_ICM42607X_SENSOR_MODE_OFF, false,
								   NULL);
	if (ret)
		goto out_unlock;

	if (!IS_ERR(st->vddio_supply))
		regulator_disable(st->vddio_supply);

	out_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static int __maybe_unused inv_icm42607x_runtime_resume(struct device *dev)
{
	struct inv_icm42607x_state *st = dev_get_drvdata(dev);
	int ret = 0;

	if (!st) // Check if driver data exists
		return 0;

	mutex_lock(&st->lock);

	if (!IS_ERR(st->vddio_supply)) {
		ret = inv_icm42607x_enable_regulator_vddio(st);
		if (ret)
			goto out_unlock;
	}

	out_unlock:
	mutex_unlock(&st->lock);
	return ret;
}

/* Sensors are enabled by iio devices, no need to turn them back on here. */
const struct dev_pm_ops inv_icm42607x_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(inv_icm42607x_suspend, inv_icm42607x_resume)
	SET_RUNTIME_PM_OPS(inv_icm42607x_runtime_suspend,
					   inv_icm42607x_runtime_resume, NULL)
};
EXPORT_SYMBOL_GPL(inv_icm42607x_pm_ops);

MODULE_AUTHOR("SylveonDeko");
MODULE_DESCRIPTION("InvenSense ICM-42607x core driver");
MODULE_LICENSE("GPL");
