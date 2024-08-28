/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bno055

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#include "bno055.h"

LOG_MODULE_REGISTER(BNO055, CONFIG_SENSOR_LOG_LEVEL);

struct bno055_config {
	struct i2c_dt_spec i2c_bus;
	uint8_t use_xtal;
};

struct bno055_data {
	uint8_t current_page;
	enum OperatingMode mode;
	struct unit_config units;

	struct vector3_data acc;
	struct vector3_data mag;
	struct vector3_data gyr;
};

static int bno055_set_config(const struct device *dev, enum OperatingMode mode)
{
	struct bno055_data *data = dev->data;
	if (data->mode == mode) {
		return 0;
	}

	const struct bno055_config *config = dev->config;
	int err;

	/* Switch to Page 0 */
	if (data->current_page != PAGE_ZERO) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, PAGE_ZERO);
		if (err < 0) {
			return err;
		}
		data->current_page = PAGE_ZERO;
	}

	if (data->mode != CONFIG_MODE) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE, CONFIG_MODE);
		if (err < 0) {
			return err;
		}
		k_sleep(K_MSEC(BNO055_TIMING_SWITCH_FROM_ANY));
	}

	if (mode == CONFIG_MODE) {
		data->mode = mode;
		return 0;
	}

	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE, mode);
	if (err < 0) {
		return err;
	}
	k_sleep(K_MSEC(BNO055_TIMING_SWITCH_FROM_CONFIG));

	data->mode = mode;
	return 0;
}

static int bno055_vector3_fetch(const struct device *dev, const uint8_t data_register, struct vector3_data *data)
{
	const struct bno055_config *config = dev->config;
	int8_t regs[6];

	int err = i2c_burst_read_dt(&config->i2c_bus, data_register, regs, sizeof(regs));
	if (err < 0) {
		return err;
	}
	data->x = (regs[1] << 8) | (0xFF & regs[0]);
	data->y = (regs[3] << 8) | (0xFF & regs[2]);
	data->z = (regs[5] << 8) | (0xFF & regs[4]);

	return 0;
}

static int bno055_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	int err;

	switch (chan)
	{
		case SENSOR_CHAN_ALL:
			if (attr == SENSOR_ATTR_CONFIGURATION) {
				switch (val->val1)
				{
					case CONFIG_MODE:
						err = bno055_set_config(dev, CONFIG_MODE);
						if (err < 0) return err;
						break;

					case ACC_ONLY:
						err = bno055_set_config(dev, ACC_ONLY);
						if (err < 0) return err;
						break;

					case MAG_ONLY:
						err = bno055_set_config(dev, MAG_ONLY);
						if (err < 0) return err;
						break;
					
					case GYRO_ONLY:
						err = bno055_set_config(dev, GYRO_ONLY);
						if (err < 0) return err;
						break;

					case ACC_MAG:
						err = bno055_set_config(dev, ACC_MAG);
						if (err < 0) return err;
						break;

					case ACC_GYRO:
						err = bno055_set_config(dev, ACC_GYRO);
						if (err < 0) return err;
						break;

					case MAG_GYRO:
						err = bno055_set_config(dev, MAG_GYRO);
						if (err < 0) return err;
						break;

					case ACC_MAG_GYRO:
						err = bno055_set_config(dev, ACC_MAG_GYRO);
						if (err < 0) return err;
						break;

					case IMU:
						err = bno055_set_config(dev, IMU);
						if (err < 0) return err;
						break;

					case COMPASS:
						err = bno055_set_config(dev, COMPASS);
						if (err < 0) return err;
						break;

					case M4G:
						err = bno055_set_config(dev, M4G);
						if (err < 0) return err;
						break;

					case NDOF_FMC_OFF:
						err = bno055_set_config(dev, NDOF_FMC_OFF);
						if (err < 0) return err;
						break;

					case NDOF:
						err = bno055_set_config(dev, NDOF);
						if (err < 0) return err;
						break;
					
					default:
						return -ENOTSUP;
				}
			}
			break;
		
		default:
			break;
	}
	return 0;
}

static int bno055_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bno055_data *data = dev->data;
	const struct bno055_config *config = dev->config;
	int err;

	/* Switch to Page 0 */
	if (data->current_page != PAGE_ZERO) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, PAGE_ZERO);
		if (err < 0) {
			return err;
		}
		data->current_page = PAGE_ZERO;
	}

	switch (data->mode)
	{
		case CONFIG_MODE:
			LOG_INF("CONFIG Mode no sample");
			break;

		case ACC_ONLY:
			LOG_INF("ACC fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_ACC_DATA, &data->acc);
			if (err < 0) {
				return err;
			}
			break;

		case MAG_ONLY:
			LOG_INF("MAG fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			break;

		case GYRO_ONLY:
			LOG_INF("GYR fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GYR_DATA, &data->gyr);
			if (err < 0) {
				return err;
			}
			break;

		case ACC_MAG:
			LOG_INF("ACC_MAG fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_ACC_DATA, &data->acc);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			break;

		case ACC_GYRO:
			LOG_INF("ACC_GYRO fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_ACC_DATA, &data->acc);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GYR_DATA, &data->gyr);
			if (err < 0) {
				return err;
			}
			break;

		case MAG_GYRO:
			LOG_INF("MAG_GYRO fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GYR_DATA, &data->gyr);
			if (err < 0) {
				return err;
			}
			break;

		case ACC_MAG_GYRO:
			LOG_INF("ACC_MAG_GYRO fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_ACC_DATA, &data->acc);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GYR_DATA, &data->gyr);
			if (err < 0) {
				return err;
			}
			break;

		case IMU:
			LOG_INF("MAG fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			break;

		case COMPASS:
			LOG_INF("MAG fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			break;

		case M4G:
			LOG_INF("MAG fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			break;

		case NDOF_FMC_OFF:
			LOG_INF("MAG fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			break;

		case NDOF:
			LOG_INF("MAG fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			break;
		
		default:
			LOG_WRN("BNO055 Not in Computation Mode!!");
			return -ENOTSUP;
	}

	return 0;
}

static int bno055_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bno055_data *data = dev->data;

	switch (chan)
	{
		case SENSOR_CHAN_ACCEL_X:
			(val)->val1 = data->acc.x / AccelerationConvertion[data->units.acceleration];
			(val)->val2 = (1000000 / AccelerationConvertion[data->units.acceleration]) * (data->acc.x - (val)->val1 * AccelerationConvertion[data->units.acceleration]);
			break;

		case SENSOR_CHAN_ACCEL_Y:
			(val)->val1 = data->acc.y / AccelerationConvertion[data->units.acceleration];
			(val)->val2 = (1000000 / AccelerationConvertion[data->units.acceleration]) * (data->acc.y - (val)->val1 * AccelerationConvertion[data->units.acceleration]);
			break;

		case SENSOR_CHAN_ACCEL_Z:
			(val)->val1 = data->acc.z / AccelerationConvertion[data->units.acceleration];
			(val)->val2 = (1000000 / AccelerationConvertion[data->units.acceleration]) * (data->acc.z - (val)->val1 * AccelerationConvertion[data->units.acceleration]);
			break;

		case SENSOR_CHAN_ACCEL_XYZ:
			(val)->val1 = data->acc.x / AccelerationConvertion[data->units.acceleration];
			(val)->val2 = (1000000 / AccelerationConvertion[data->units.acceleration]) * (data->acc.x - (val)->val1 * AccelerationConvertion[data->units.acceleration]);
			(val+1)->val1 = data->acc.y / AccelerationConvertion[data->units.acceleration];
			(val+1)->val2 = (1000000 / AccelerationConvertion[data->units.acceleration]) * (data->acc.y - (val+1)->val1 * AccelerationConvertion[data->units.acceleration]);
			(val+2)->val1 = data->acc.z / AccelerationConvertion[data->units.acceleration];
			(val+2)->val2 = (1000000 / AccelerationConvertion[data->units.acceleration]) * (data->acc.z - (val+2)->val1 * AccelerationConvertion[data->units.acceleration]);
			break;
		
		default:
			return -ENOTSUP;
	}

	return 0;
}

static int bno055_init(const struct device *dev)
{
	const struct bno055_config *config = dev->config;
	struct bno055_data *data = dev->data;

	if (!i2c_is_ready_dt(&config->i2c_bus)) {
		LOG_ERR("I2C bus not ready!!");
		return -ENODEV;
	}

	LOG_INF("CONFIG");
	LOG_INF("USE XTAL [%d]", config->use_xtal);
	int err;

	/* Switch to Page 0 */
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, PAGE_ZERO);
	if (err < 0) {
		return err;
	}
	data->current_page = PAGE_ZERO;
	
	/* Send Reset Command */ // GOOD IDEA??
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER, BNO055_COMMAND_RESET);
	if (err < 0) {
		return err;
	}
	data->mode = CONFIG_MODE;
	k_sleep(K_MSEC(BNO055_TIMING_RESET_CONFIG));

	/* Check for chip id to validate the power on of the sensor */
	uint8_t chip_id[1];
	err = i2c_burst_read_dt(&config->i2c_bus, BNO055_REGISTER_CHIP_ID, chip_id, sizeof(chip_id));
	if (err < 0) {
		return err;
	}
	LOG_INF("CHIP ID [%d]", chip_id[0]);

	if (chip_id[0] != BNO055_CHIP_ID) {
		LOG_WRN("BNO055 Not Ready yet!!");
		k_sleep(K_MSEC(BNO055_TIMING_RESET_CONFIG));
		err = i2c_burst_read_dt(&config->i2c_bus, BNO055_REGISTER_CHIP_ID, chip_id, sizeof(chip_id));
		if (err < 0) {
			return err;
		}
		if (chip_id[0] != BNO055_CHIP_ID) {
            return -ENODEV;
        }
	}

	/* Configure Unit according to Zephyr */
	data->units.orientation = WINDOWS;
	data->units.temp = CELSIUS;
	data->units.euler = RADIANS;
	data->units.rotation = RPS;
	data->units.acceleration = MS_2;
	uint8_t selection = (data->units.orientation << 7)	|
						(data->units.temp << 4)			|
						(data->units.euler << 2)		|
						(data->units.rotation << 1)		|
						(data->units.acceleration << 0);
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_UNIT_SELECT, selection);
	if (err < 0) {
		return err;
	}

	if (config->use_xtal) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER, BNO055_COMMAND_XTAL);
		if (err < 0) {
			return err;
		}
	}

	// /!\ To DO: Check BIST result of Power on Self Test

	return 0;
}

static const struct sensor_driver_api bno055_driver_api = {
	.attr_set = bno055_attr_set,
	.sample_fetch = bno055_sample_fetch,
	.channel_get = bno055_channel_get,
};

#define BNO055_INIT(n)                                                                             \
	static struct bno055_config bno055_config_##n = {                                             \
		.i2c_bus = I2C_DT_SPEC_INST_GET(n),                                                    \
		.use_xtal = DT_INST_PROP(n, use_xtal),                                                    \
	};                                                                                         \
	static struct bno055_data bno055_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, bno055_init, NULL, &bno055_data_##n, &bno055_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &bno055_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BNO055_INIT)
