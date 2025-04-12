// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023 InvenSense, Inc.
 * Copyright (C) 2025 SylveonDeko
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/property.h>

#include "inv_icm42607x.h"

static int inv_icm42607x_spi_bus_setup(struct inv_icm42607x_state *st)
{
    unsigned int mask, val;
    int ret;

    ret = regmap_update_bits(st->map, INV_ICM42607X_REG_DEVICE_CONFIG,
                             INV_ICM42607X_DEVICE_CONFIG_SPI_AP_4WIRE,
                             INV_ICM42607X_DEVICE_CONFIG_SPI_AP_4WIRE);
    if (ret)
        return ret;

    ret = regmap_update_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG1,
                             INV_ICM42607X_INTF_CONFIG1_I3C_DDR_EN |
                             INV_ICM42607X_INTF_CONFIG1_I3C_SDR_EN,
                             0);
    if (ret)
        return ret;

    mask = INV_ICM42607X_DRIVE_CONFIG3_SPI_MASK;
    val = INV_ICM42607X_DRIVE_CONFIG3_SPI(INV_ICM42607X_SLEW_RATE_INF_2NS);
    ret = regmap_update_bits(st->map, INV_ICM42607X_REG_DRIVE_CONFIG3,
                             mask, val);
    if (ret)
        return ret;

    return regmap_update_bits(st->map, INV_ICM42607X_REG_INTF_CONFIG0,
                              INV_ICM42607X_INTF_CONFIG0_UI_SIFS_CFG_MASK,
                              INV_ICM42607X_INTF_CONFIG0_UI_SIFS_CFG_I2C_DIS);
}

static int inv_icm42607x_find_compat(struct spi_device *spi,
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

static int inv_icm42607x_probe(struct spi_device *spi)
{
    const struct spi_driver *sdrv = to_spi_driver(spi->dev.driver);
    const struct of_device_id *of_id = sdrv->driver.of_match_table;
    const struct spi_device_id *id = spi_get_device_id(spi);
    bool multi_driver = false;
    enum inv_icm42607x_chip chip;
    struct regmap *regmap;
    int res = -ENODEV;
    int hw_id = -ENODEV;
    int ret;

    spi->mode = SPI_MODE_3;
    spi->bits_per_word = 8;
    if (spi->max_speed_hz > 24000000)
        spi->max_speed_hz = 24000000;

    ret = spi_setup(spi);
    if (ret) {
        dev_err(&spi->dev, "SPI setup failed: %d\n", ret);
        return ret;
    }

    if (!id) {
        // Only check modalias if we have a device tree node or explicit multi-driver modalias
        if (spi->dev.of_node || (spi->modalias && !strcmp(spi->modalias, "multi-driver"))) {
            if (spi->modalias && !strcmp(spi->modalias, "multi-driver")) {
                multi_driver = true;
            }
            // If we have DT node, multi_driver might still be set if core_probe fails later
            // and we need to iterate through DT compatibles. Let find_compat handle DT case.
        } else {
            dev_err(&spi->dev, "Failed to get spi id: %s\n",
                    spi->modalias ? spi->modalias : "NULL");
            return -ENODEV;
        }
    } else {
        hw_id = id->driver_data;
        dev_info(&spi->dev, "Probing via SPI ID: %s\n", id->name);
    }

    regmap = devm_regmap_init_spi(spi, &inv_icm42607x_regmap_config);
    if (IS_ERR(regmap)) {
        dev_err(&spi->dev, "Failed to register spi regmap %ld\n",
                PTR_ERR(regmap));
        return PTR_ERR(regmap);
    }

    try_next:
    // Determine hw_id from DT if needed (multi_driver or first pass with DT node)
    if (spi->dev.of_node && (multi_driver || hw_id < 0)) {
        if (!of_id) {
            dev_err(&spi->dev, "OF match table is NULL\n");
            return -EINVAL;
        }
        hw_id = inv_icm42607x_find_compat(spi, &of_id);
        if (hw_id < 0)
            return hw_id; // Error already printed
        // If find_compat succeeded, ensure multi_driver is set for the retry logic
        multi_driver = true;
    } else if (hw_id < 0) {
        // This case should ideally not be reached if !id check above worked correctly
        dev_err(&spi->dev, "Could not determine hardware ID\n");
        return -ENODEV;
    }


    chip = (enum inv_icm42607x_chip)hw_id;

    res = inv_icm42607x_core_probe(regmap, chip, spi->irq,
                                   inv_icm42607x_spi_bus_setup);

    if (multi_driver) {
        if (res && of_id && of_id->compatible && of_id->compatible[0]) {
            dev_info(&spi->dev, "Core probe failed for %d (%d), trying next compatible\n", chip, res);
            hw_id = -ENODEV; // Reset hw_id to force re-check in find_compat
            goto try_next;
        }
        if (!res)
            dev_info(&spi->dev, "Probed successfully via multi-driver/DT\n");
    }

    return res;
}

static const struct of_device_id inv_icm42607x_of_matches[] = {
    {
        .compatible = "invensense,icm42607p",
        .data = (void *)INV_CHIP_ICM42607P,
    },
    {
        .compatible = "invensense,icm42670",
        .data = (void *)INV_CHIP_ICM42607,
    },
    {}
};
MODULE_DEVICE_TABLE(of, inv_icm42607x_of_matches);

static const struct spi_device_id inv_icm42607x_spi_id_table[] = {
    { "icm42607p", INV_CHIP_ICM42607P },
    { "icm42670", INV_CHIP_ICM42607 },
    {},
};
MODULE_DEVICE_TABLE(spi, inv_icm42607x_spi_id_table);

static struct spi_driver inv_icm42607x_driver = {
    .driver = {
        .name = "inv-icm42607x-spi",
        .of_match_table = of_match_ptr(inv_icm42607x_of_matches),
        .pm = &inv_icm42607x_pm_ops,
    },
    .probe = inv_icm42607x_probe,
    .id_table = inv_icm42607x_spi_id_table,
};
module_spi_driver(inv_icm42607x_driver);

MODULE_AUTHOR("SylveonDeko");
MODULE_DESCRIPTION("InvenSense ICM-42607x SPI driver");
MODULE_LICENSE("GPL");
