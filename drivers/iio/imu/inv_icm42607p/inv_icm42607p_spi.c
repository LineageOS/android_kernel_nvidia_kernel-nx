// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2023 InvenSense, Inc.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/property.h>

#include "inv_icm42607p.h"

static int inv_icm42607p_spi_bus_setup(struct inv_icm42607p_state *st)
{
    unsigned int mask, val;
    int ret;

    /* Configure for 4-wire SPI mode */
    ret = regmap_update_bits(st->map, INV_ICM42607P_REG_DEVICE_CONFIG,
                             INV_ICM42607P_DEVICE_CONFIG_SPI_AP_4WIRE,
                             INV_ICM42607P_DEVICE_CONFIG_SPI_AP_4WIRE);
    if (ret)
        return ret;

    /* Disable I3C interfaces */
    ret = regmap_update_bits(st->map, INV_ICM42607P_REG_INTF_CONFIG1,
                             INV_ICM42607P_INTF_CONFIG1_I3C_DDR_EN |
                             INV_ICM42607P_INTF_CONFIG1_I3C_SDR_EN,
                             0);
    if (ret)
        return ret;

    /* Set slew rates for SPI */
    mask = INV_ICM42607P_DRIVE_CONFIG3_SPI_MASK;
    val = INV_ICM42607P_DRIVE_CONFIG3_SPI(INV_ICM42607P_SLEW_RATE_INF_2NS);
    ret = regmap_update_bits(st->map, INV_ICM42607P_REG_DRIVE_CONFIG3,
                             mask, val);
    if (ret)
        return ret;

    /* Disable I2C interface */
    return regmap_update_bits(st->map, INV_ICM42607P_REG_INTF_CONFIG0,
                              INV_ICM42607P_INTF_CONFIG0_UI_SIFS_CFG_MASK,
                              INV_ICM42607P_INTF_CONFIG0_UI_SIFS_CFG_I2C_DIS);
}

static int inv_icm42607p_find_compat(struct spi_device *spi,
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

static int inv_icm42607p_probe(struct spi_device *spi)
{
    const struct spi_driver *sdrv = to_spi_driver(spi->dev.driver);
    const struct of_device_id *of_id = sdrv->driver.of_match_table;
    const struct spi_device_id *id = spi_get_device_id(spi);
    bool multi_driver = false;
    enum inv_icm42607p_chip chip;
    struct regmap *regmap;
    int res, hw_id, ret;

    /* Set SPI mode */
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
        if (!strcmp(spi->modalias, "multi-driver")) {
            multi_driver = true;
        } else {
            dev_err(&spi->dev, "Failed to get spi id: %s\n",
                    spi->modalias);
            return -ENODEV;
        }
    } else {
        hw_id = id->driver_data;
    }

    regmap = devm_regmap_init_spi(spi, &inv_icm42607p_regmap_config);
    if (IS_ERR(regmap)) {
        dev_err(&spi->dev, "Failed to register spi regmap %d\n",
                (int)PTR_ERR(regmap));
        return PTR_ERR(regmap);
    }

    try_next:
    if (multi_driver) {
        hw_id = inv_icm42607p_find_compat(spi, &of_id);
        if (hw_id < 0)
            return hw_id;
    }

    chip = (enum inv_icm42607p_chip)hw_id;

    res = inv_icm42607p_core_probe(regmap, chip, spi->irq,
                                   inv_icm42607p_spi_bus_setup);

    if (multi_driver) {
        if (res)
            goto try_next;
        dev_info(&spi->dev, "Probed\n");
    }

    return res;
}

static const struct of_device_id inv_icm42607p_of_matches[] = {
    {
        .compatible = "invensense,icm42607p",
        .data = (void *)INV_CHIP_ICM42607P,
    },
    {}
};
MODULE_DEVICE_TABLE(of, inv_icm42607p_of_matches);

static const struct spi_device_id inv_icm42607p_spi_id_table[] = {
    { "icm42607p", INV_CHIP_ICM42607P },
    {},
};
MODULE_DEVICE_TABLE(spi, inv_icm42607p_spi_id_table);

static struct spi_driver inv_icm42607p_driver = {
    .driver = {
        .name = "inv-icm42607p-spi",
        .of_match_table = of_match_ptr(inv_icm42607p_of_matches),
        .pm = &inv_icm42607p_pm_ops,
    },
    .probe = inv_icm42607p_probe,
    .id_table = inv_icm42607p_spi_id_table,
};
module_spi_driver(inv_icm42607p_driver);

MODULE_AUTHOR("InvenSense, Inc.");
MODULE_DESCRIPTION("InvenSense ICM-42607P SPI driver");
MODULE_LICENSE("GPL");
