/*
 * Copyright (c) 2019 Centaur Analytics, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_tmp11x_rs

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/sensor/tmp11x.h>
#include <zephyr/dt-bindings/sensor/tmp11x.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

/* ---- FFI interface to Rust ---- */
struct tmp11x_rs_ctx {
    const struct device *i2c;
    uint16_t i2c_addr;
    /* Additional state can be stored here; keep it POD for FFI. */
};

struct tmp11x_data {
    uint16_t sample;
    uint16_t id;
};

struct tmp11x_dev_config {
    struct i2c_dt_spec bus;
    uint16_t odr;
    uint16_t oversampling;
    bool alert_pin_polarity;
    bool alert_mode;
    bool alert_dr_sel;
    bool store_attr_values;
};

#define RESET_MIN_BUSY_MS 2

LOG_MODULE_REGISTER(TMP11X, CONFIG_SENSOR_LOG_LEVEL);

int tmp11x_reg_read(const struct device *dev, uint8_t reg, uint16_t *val)
{
    const struct tmp11x_dev_config *cfg = dev->config;

    if (i2c_burst_read_dt(&cfg->bus, reg, (uint8_t *)val, 2) < 0) {
        return -EIO;
    }

    *val = sys_be16_to_cpu(*val);

    return 0;
}

int tmp11x_reg_read_wrapper(void *ptr, uint8_t reg, uint16_t *val)
{
    return tmp11x_reg_read((const struct device *)ptr, reg, val);
}

static int tmp11x_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    /*
     * For simplicity this is a sub.
     */

    return 0;
}

static int tmp11x_attr_set(const struct device *dev, enum sensor_channel chan,
                           enum sensor_attribute attr, const struct sensor_value *val)
{
    // Stub implementation
    return 0;
}

static int tmp11x_attr_get(const struct device *dev, enum sensor_channel chan,
                           enum sensor_attribute attr, struct sensor_value *val)
{
    // Stub implementation
    return 0;
}

extern int tmp11x_rs_channel_get(const void *dev, int chan, struct sensor_value *val);

static int tmp11x_channel_get(const struct device *dev, enum sensor_channel chan,
                  struct sensor_value *val)
{
    int result = tmp11x_rs_channel_get((const void *)dev, (int)chan, val);

    return 0;
}

static DEVICE_API(sensor, tmp11x_rs_driver_api) = {
    .attr_set = tmp11x_attr_set,
    .attr_get = tmp11x_attr_get,
    .sample_fetch = tmp11x_sample_fetch,
    .channel_get = tmp11x_channel_get,
};

static int tmp11x_rs_init(const struct device *dev)
{
    // struct tmp11x_data *drv_data = dev->data;
    const struct tmp11x_dev_config *cfg = dev->config;

    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("I2C dev %s not ready", cfg->bus.bus->name);
        return -EINVAL;
    }

    return 0;
}

#ifdef CONFIG_TMP11X_RS_TRIGGER
#define DEFINE_TMP11X_RS_TRIGGER(_num)                                                             \
    .alert_gpio = GPIO_DT_SPEC_INST_GET_OR(_num, alert_gpios, {}),
#else
#define DEFINE_TMP11X_RS_TRIGGER(_num)
#endif

#define DEFINE_TMP11X_RS(_num)                                                                     \
    static struct tmp11x_data tmp11x_rs_data_##_num;                                           \
    static const struct tmp11x_dev_config tmp11x_rs_config_##_num = {                          \
        .bus = I2C_DT_SPEC_INST_GET(_num),                                                 \
        .odr = DT_INST_PROP(_num, odr),                                                    \
        .oversampling = DT_INST_PROP(_num, oversampling),                                  \
        .alert_pin_polarity = DT_INST_PROP(_num, alert_polarity),                          \
        .alert_mode = DT_INST_PROP(_num, alert_mode),                                      \
        .alert_dr_sel = DT_INST_PROP(_num, alert_dr_sel),                                  \
        .store_attr_values = DT_INST_PROP(_num, store_attr_values),                        \
        DEFINE_TMP11X_RS_TRIGGER(_num)};                                                   \
                                                                                                   \
    SENSOR_DEVICE_DT_INST_DEFINE(_num, tmp11x_rs_init, PM_DEVICE_DT_INST_GET(_num),            \
                     &tmp11x_rs_data_##_num, &tmp11x_rs_config_##_num,             \
                     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                     \
                     &tmp11x_rs_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_TMP11X_RS)
