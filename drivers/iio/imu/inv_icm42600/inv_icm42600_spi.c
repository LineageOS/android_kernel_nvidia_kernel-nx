// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020 InvenSense, Inc.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/property.h>

#include "inv_icm42600.h"

static int inv_icm42600_spi_bus_setup(struct inv_icm42600_state *st)
{
	unsigned int mask, val;
	int ret;

	/* setup interface registers */
	val = INV_ICM42600_INTF_CONFIG6_I3C_EN |
	      INV_ICM42600_INTF_CONFIG6_I3C_SDR_EN |
	      INV_ICM42600_INTF_CONFIG6_I3C_DDR_EN;
	ret = regmap_update_bits(st->map, INV_ICM42600_REG_INTF_CONFIG6,
				 INV_ICM42600_INTF_CONFIG6_MASK, val);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->map, INV_ICM42600_REG_INTF_CONFIG4,
				 INV_ICM42600_INTF_CONFIG4_I3C_BUS_ONLY, 0);
	if (ret)
		return ret;

	/* set slew rates for I2C and SPI */
	mask = INV_ICM42600_DRIVE_CONFIG_I2C_MASK |
	       INV_ICM42600_DRIVE_CONFIG_SPI_MASK;
	val = INV_ICM42600_DRIVE_CONFIG_I2C(INV_ICM42600_SLEW_RATE_20_60NS) |
	      INV_ICM42600_DRIVE_CONFIG_SPI(INV_ICM42600_SLEW_RATE_INF_2NS);
	ret = regmap_update_bits(st->map, INV_ICM42600_REG_DRIVE_CONFIG,
				 mask, val);
	if (ret)
		return ret;

	/* disable i2c bus */
	return regmap_update_bits(st->map, INV_ICM42600_REG_INTF_CONFIG0,
				  INV_ICM42600_INTF_CONFIG0_UI_SIFS_CFG_MASK,
				  INV_ICM42600_INTF_CONFIG0_UI_SIFS_CFG_I2C_DIS);
}

static int inv_icm42600_find_compat(struct spi_device *spi,
	const struct of_device_id **of_id)
{
	const struct of_device_id *id;
	int hw_id;

	while (true) {
		id = *of_id;
		if (!id->compatible || !id->compatible[0]) {
			dev_err(&spi->dev, "Failed to probe compatible device\n");
			return -ENODEV;
		}
		if (of_device_is_compatible(spi->dev.of_node, id->compatible)) {
			dev_info(&spi->dev, "Probing %s\n", id->compatible);
			hw_id = (int)(uintptr_t)id->data;
			break;
		}
		(*of_id)++;
	}
	(*of_id)++;

	return hw_id;
}

static int inv_icm42600_probe(struct spi_device *spi)
{
    const struct spi_driver *sdrv;
    const struct of_device_id *of_id;
    const struct spi_device_id *id;
    bool multi_driver = false;
    enum inv_icm42600_chip chip;
    struct regmap *regmap;
    int res, hw_id;

    // Check spi pointer itself (should never be NULL here)
    if (!spi) {
        pr_err("inv_icm42600_probe: FATAL: spi pointer is NULL!\n");
        return -EINVAL; // Should not happen
    }
    dev_dbg(&spi->dev, "Probe START: spi=%p\n", spi);

    // Check essential pointers within spi and spi->dev
    dev_dbg(&spi->dev, "Probe: &spi->dev=%p\n", &spi->dev);
    dev_dbg(&spi->dev, "Probe: spi->dev.driver=%p\n", spi->dev.driver);
    dev_dbg(&spi->dev, "Probe: spi->dev.of_node=%p\n", spi->dev.of_node);
    dev_dbg(&spi->dev, "Probe: spi->modalias=%s\n", spi->modalias ? spi->modalias : "NULL");
    dev_dbg(&spi->dev, "Probe: spi->irq=%d\n", spi->irq);

    sdrv = to_spi_driver(spi->dev.driver);
    dev_dbg(&spi->dev, "Probe: sdrv=%p\n", sdrv);
    if (!sdrv) {
         dev_err(&spi->dev, "Probe: sdrv (driver) pointer is NULL!\n");
         return -EINVAL; // Should not happen
    } else {
        dev_info(&spi->dev, "Probe: sdrv->driver.of_match_table=%p\n", sdrv->driver.of_match_table);
        // Check if of_match_table is NULL *before* assigning to of_id
        if (!sdrv->driver.of_match_table) {
            dev_warn(&spi->dev, "Probe: sdrv->driver.of_match_table is NULL!\n");
        }
    }
    of_id = sdrv->driver.of_match_table;
    dev_dbg(&spi->dev, "Probe: of_id=%p\n", of_id);


    id = spi_get_device_id(spi);
    dev_dbg(&spi->dev, "Probe: id=%p\n", id);


    if (!id) {
        dev_dbg(&spi->dev, "Probe: No SPI device ID found, checking modalias.\n");
        if (spi->modalias) { // Check modalias before strcmp
             if (!strcmp(spi->modalias, "multi-driver")) {
                 dev_dbg(&spi->dev, "Probe: multi-driver modalias detected.\n");
                 multi_driver = true;
             } else {
                 dev_err(&spi->dev, "Failed to get spi id: %s\n",
                         spi->modalias);
                 return -ENODEV;
             }
        } else {
             dev_err(&spi->dev, "Probe: spi->modalias is NULL when ID is NULL!\n");
             return -ENODEV; // Can't identify device
        }
    } else {
        dev_dbg(&spi->dev, "Probe: Got SPI device ID: name=%s, driver_data=0x%lx\n", id->name, id->driver_data);
        hw_id = id->driver_data;
    }

    dev_dbg(&spi->dev, "Probe: Calling devm_regmap_init_spi...\n");
    regmap = devm_regmap_init_spi(spi, &inv_icm42600_regmap_config);
    // Check regmap BEFORE IS_ERR. IS_ERR handles NULL and error codes (-PTR_MAX_ERR .. -1)
    // Printing it directly might show NULL or an ERR_PTR value.
    dev_dbg(&spi->dev, "Probe: devm_regmap_init_spi returned: %p\n", regmap);
    if (IS_ERR(regmap)) {
        dev_err(&spi->dev, "Failed to register spi regmap %ld\n", // Use %ld for long
                PTR_ERR(regmap));
        return PTR_ERR(regmap);
    }
    dev_dbg(&spi->dev, "Probe: Regmap initialized successfully.\n");


try_next: // Label for multi-driver loop
    if (multi_driver) {
        dev_dbg(&spi->dev, "Probe: multi-driver: finding compatible, of_id=%p\n", of_id);
        // Check of_id before passing it to the function
        if (!of_id) {
             dev_err(&spi->dev, "Probe: multi-driver: of_id is NULL before calling find_compat!\n");
             return -EINVAL; // Cannot proceed
        }
        hw_id = inv_icm42600_find_compat(spi, &of_id);
        dev_dbg(&spi->dev, "Probe: multi-driver: find_compat returned hw_id=%d, new of_id=%p\n", hw_id, of_id);
        if (hw_id < 0)
            return hw_id;
    }

    chip = (enum inv_icm42600_chip)hw_id;
    dev_dbg(&spi->dev, "Probe: Determined chip type: %d. Calling core_probe (irq=%d)...\n", (int)chip, spi->irq);

    res = inv_icm42600_core_probe(regmap, chip, spi->irq,
                                 inv_icm42600_spi_bus_setup);

    dev_dbg(&spi->dev, "Probe: core_probe returned: %d\n", res);

    if (multi_driver) {
        if (res) {
             dev_dbg(&spi->dev, "Probe: multi-driver: core_probe failed (%d), trying next compatible...\n", res);
             goto try_next;
        }
        dev_dbg(&spi->dev, "Probed successfully via multi-driver\n");
    }

    dev_dbg(&spi->dev, "Probe END: result=%d\n", res);
    return res;
}

static const struct of_device_id inv_icm42600_of_matches[] = {
	{
		.compatible = "invensense,icm42600",
		.data = (void *)INV_CHIP_ICM42600,
	}, {
		.compatible = "invensense,icm40607",
		.data = (void *)INV_CHIP_ICM40607,
	}, {
		.compatible = "invensense,icm42602",
		.data = (void *)INV_CHIP_ICM42602,
	}, {
		.compatible = "invensense,icm42605",
		.data = (void *)INV_CHIP_ICM42605,
	}, {
		.compatible = "invensense,icm42622",
		.data = (void *)INV_CHIP_ICM42622,
	},
	{}
};
MODULE_DEVICE_TABLE(of, inv_icm42600_of_matches);

static const struct spi_device_id inv_icm42600_spi_id_table[] = {
	{ "icm42600", INV_CHIP_ICM42600 },
	{ "icm40607", INV_CHIP_ICM40607 },
	{ "icm42602", INV_CHIP_ICM42602 },
	{ "icm42605", INV_CHIP_ICM42605 },
	{ "icm42622", INV_CHIP_ICM42622 },
	{},
};
MODULE_DEVICE_TABLE(spi, inv_icm42600_spi_id_table);

static struct spi_driver inv_icm42600_driver = {
	.driver = {
		.name = "inv-icm42600-spi",
		.of_match_table = of_match_ptr(inv_icm42600_of_matches),
		.pm = &inv_icm42600_pm_ops,
	},
	.probe = inv_icm42600_probe,
	.id_table = inv_icm42600_spi_id_table,
};
module_spi_driver(inv_icm42600_driver);

MODULE_AUTHOR("InvenSense, Inc.");
MODULE_DESCRIPTION("InvenSense ICM-426xx SPI driver");
MODULE_LICENSE("GPL");
