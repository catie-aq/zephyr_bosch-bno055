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
	enum PowerMode power;

	struct vector3_data acc;
	struct vector3_data mag;
	struct vector3_data gyr;

	struct vector3_data eul;
	struct vector4_data qua;
	struct vector3_data lia;
	struct vector3_data grv;

	struct calib_data calib;
};

static int bno055_set_config(const struct device *dev, enum OperatingMode mode, bool fusion)
{
	struct bno055_data *data = dev->data;
	if (data->mode == mode) {
		return 0;
	}

	const struct bno055_config *config = dev->config;
	int err;

	/* Switch to Page 0 */
	if (data->current_page != BNO055_PAGE_ZERO) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, BNO055_PAGE_ZERO);
		if (err < 0) {
			return err;
		}
		data->current_page = BNO055_PAGE_ZERO;
	}

	if (data->mode != BNO055_MODE_CONFIG) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE, BNO055_MODE_CONFIG);
		if (err < 0) {
			return err;
		}
		k_sleep(K_MSEC(BNO055_TIMING_SWITCH_FROM_ANY));
	}

	if (mode == BNO055_MODE_CONFIG) {
		data->mode = mode;
		return 0;
	}

#if defined(CONFIG_BNO055_ACC_CUSTOM_CONFIG) || defined(CONFIG_BNO055_MAG_CUSTOM_CONFIG) || defined(CONFIG_BNO055_GYR_CUSTOM_CONFIG)
	if (!fusion) {
		/* Switch to Page 1 */
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, BNO055_PAGE_ONE);
		if (err < 0) {
			return err;
		}
		data->current_page = BNO055_PAGE_ONE;

		uint8_t reg = 0x00;
		reg = reg | BNO055_ACC_RANGE | BNO055_ACC_BANDWIDTH | BNO055_ACC_POWER;
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_ACC_CONFIG, reg);
		if (err < 0) {
			return err;
		}

		reg = 0x00 | BNO055_MAG_RATE | BNO055_MAG_MODE | BNO055_MAG_POWER;
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_MAG_CONFIG, reg);
		if (err < 0) {
			return err;
		}

		reg = 0x00 | BNO055_GYR_RANGE | BNO055_GYR_BANDWIDTH;
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_GYR_CONFIG_0, reg);
		if (err < 0) {
			return err;
		}
		reg = 0x00 | BNO055_GYR_POWER;
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_GYR_CONFIG_1, reg);
		if (err < 0) {
			return err;
		}

		/* Switch back to Page 0 */
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, BNO055_PAGE_ZERO);
		if (err < 0) {
			return err;
		}
		data->current_page = BNO055_PAGE_ZERO;
	}
#endif

	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE, mode);
	if (err < 0) {
		return err;
	}
	k_sleep(K_MSEC(BNO055_TIMING_SWITCH_FROM_CONFIG));

	data->mode = mode;
	return 0;
}

static int bno055_set_power(const struct device *dev, enum PowerMode power)
{
	struct bno055_data *data = dev->data;
	if (data->power == power) {
		return 0;
	}

	const struct bno055_config *config = dev->config;
	int err;

	enum OperatingMode mode = data->mode;
	err = bno055_set_config(dev, BNO055_MODE_CONFIG, false);
	if (err < 0) return err;

	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_POWER_MODE, power);
	if (err < 0) return err;
	data->power = power;

	err = bno055_set_config(dev, mode, mode < BNO055_MODE_IMU ? false : true);
	if (err < 0) return err;

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

static int bno055_vector4_fetch(const struct device *dev, const uint8_t data_register, struct vector4_data *data)
{
	const struct bno055_config *config = dev->config;
	int8_t regs[8];

	int err = i2c_burst_read_dt(&config->i2c_bus, data_register, regs, sizeof(regs));
	if (err < 0) {
		return err;
	}
	data->w = (regs[1] << 8) | (0xFF & regs[0]);
	data->x = (regs[3] << 8) | (0xFF & regs[2]);
	data->y = (regs[5] << 8) | (0xFF & regs[4]);
	data->z = (regs[7] << 8) | (0xFF & regs[6]);

	return 0;
}

static int bno055_calibration_fetch(const struct device *dev, struct calib_data *data)
{
	const struct bno055_config *config = dev->config;
	int8_t regs[1];

	int err = i2c_burst_read_dt(&config->i2c_bus, BNO055_REGISTER_CALIBRATION_STATUS, regs, sizeof(regs));
	if (err < 0) {
		return err;
	}
	data->sys = (regs[0] >> 6) & 0x03;
	data->gyr = (regs[0] >> 4) & 0x03;
	data->acc = (regs[0] >> 2) & 0x03;
	data->mag = (regs[0] >> 0) & 0x03;

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
					case BNO055_MODE_CONFIG:
						err = bno055_set_config(dev, BNO055_MODE_CONFIG, false);
						if (err < 0) return err;
						break;

					case BNO055_MODE_ACC_ONLY:
						err = bno055_set_config(dev, BNO055_MODE_ACC_ONLY, false);
						if (err < 0) return err;
						break;

					case BNO055_MODE_MAG_ONLY:
						err = bno055_set_config(dev, BNO055_MODE_MAG_ONLY, false);
						if (err < 0) return err;
						break;
					
					case BNO055_MODE_GYRO_ONLY:
						err = bno055_set_config(dev, BNO055_MODE_GYRO_ONLY, false);
						if (err < 0) return err;
						break;

					case BNO055_MODE_ACC_MAG:
						err = bno055_set_config(dev, BNO055_MODE_ACC_MAG, false);
						if (err < 0) return err;
						break;

					case BNO055_MODE_ACC_GYRO:
						err = bno055_set_config(dev, BNO055_MODE_ACC_GYRO, false);
						if (err < 0) return err;
						break;

					case BNO055_MODE_MAG_GYRO:
						err = bno055_set_config(dev, BNO055_MODE_MAG_GYRO, false);
						if (err < 0) return err;
						break;

					case BNO055_MODE_ACC_MAG_GYRO:
						err = bno055_set_config(dev, BNO055_MODE_ACC_MAG_GYRO, false);
						if (err < 0) return err;
						break;

					case BNO055_MODE_IMU:
						err = bno055_set_config(dev, BNO055_MODE_IMU, true);
						if (err < 0) return err;
						break;

					case BNO055_MODE_COMPASS:
						err = bno055_set_config(dev, BNO055_MODE_COMPASS, true);
						if (err < 0) return err;
						break;

					case BNO055_MODE_M4G:
						err = bno055_set_config(dev, BNO055_MODE_M4G, true);
						if (err < 0) return err;
						break;

					case BNO055_MODE_NDOF_FMC_OFF:
						err = bno055_set_config(dev, BNO055_MODE_NDOF_FMC_OFF, true);
						if (err < 0) return err;
						break;

					case BNO055_MODE_NDOF:
						err = bno055_set_config(dev, BNO055_MODE_NDOF, true);
						if (err < 0) return err;
						break;
					
					default:
						return -EINVAL;
				}
			} else if (attr == (enum sensor_attribute)BNO055_SENSOR_ATTR_POWER_MODE) {
				switch (val->val1)
				{
					case BNO055_POWER_NORMAL:
						err = bno055_set_power(dev, BNO055_POWER_NORMAL);
						if (err < 0) return err;
						break;
					
					case BNO055_POWER_LOW_POWER:
						err = bno055_set_power(dev, BNO055_POWER_LOW_POWER);
						if (err < 0) return err;
						break;

					case BNO055_POWER_SUSPEND:
						err = bno055_set_power(dev, BNO055_POWER_SUSPEND);
						if (err < 0) return err;
						break;
					
					case BNO055_POWER_INVALID:
						err = bno055_set_power(dev, BNO055_POWER_INVALID);
						if (err < 0) return err;
						break;
				
					default:
						return -EINVAL;
				}
			}
			
			break;
		
		default:
			return -ENOTSUP;
	}
	return 0;
}

static int bno055_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bno055_data *data = dev->data;
	const struct bno055_config *config = dev->config;
	int err;

	/* Switch to Page 0 */
	if (data->current_page != BNO055_PAGE_ZERO) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, BNO055_PAGE_ZERO);
		if (err < 0) {
			return err;
		}
		data->current_page = BNO055_PAGE_ZERO;
	}

	switch (data->mode)
	{
		case BNO055_MODE_CONFIG:
			LOG_INF("CONFIG Mode no sample");
			break;

		case BNO055_MODE_ACC_ONLY:
			LOG_INF("ACC fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_ACC_DATA, &data->acc);
			if (err < 0) {
				return err;
			}
			break;

		case BNO055_MODE_MAG_ONLY:
			LOG_INF("MAG fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
			if (err < 0) {
				return err;
			}
			break;

		case BNO055_MODE_GYRO_ONLY:
			LOG_INF("GYR fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GYR_DATA, &data->gyr);
			if (err < 0) {
				return err;
			}
			break;

		case BNO055_MODE_ACC_MAG:
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

		case BNO055_MODE_ACC_GYRO:
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

		case BNO055_MODE_MAG_GYRO:
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

		case BNO055_MODE_ACC_MAG_GYRO:
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

		case BNO055_MODE_IMU:
			LOG_INF("IMU fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_EUL_DATA, &data->eul);
			if (err < 0) {
				return err;
			}
			err = bno055_vector4_fetch(dev, BNO055_REGISTER_QUA_DATA, &data->qua);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_LIA_DATA, &data->lia);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GRV_DATA, &data->grv);
			if (err < 0) {
				return err;
			}
			err = bno055_calibration_fetch(dev, &data->calib);
			if (err < 0) {
				return err;
			}
			break;

		case BNO055_MODE_COMPASS:
			LOG_INF("COMPASS fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_EUL_DATA, &data->eul);
			if (err < 0) {
				return err;
			}
			err = bno055_vector4_fetch(dev, BNO055_REGISTER_QUA_DATA, &data->qua);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_LIA_DATA, &data->lia);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GRV_DATA, &data->grv);
			if (err < 0) {
				return err;
			}
			err = bno055_calibration_fetch(dev, &data->calib);
			if (err < 0) {
				return err;
			}
			break;

		case BNO055_MODE_M4G:
			LOG_INF("M4G fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_EUL_DATA, &data->eul);
			if (err < 0) {
				return err;
			}
			err = bno055_vector4_fetch(dev, BNO055_REGISTER_QUA_DATA, &data->qua);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_LIA_DATA, &data->lia);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GRV_DATA, &data->grv);
			if (err < 0) {
				return err;
			}
			err = bno055_calibration_fetch(dev, &data->calib);
			if (err < 0) {
				return err;
			}
			break;

		case BNO055_MODE_NDOF_FMC_OFF:
			LOG_INF("NDOF_FMC_OFF fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_EUL_DATA, &data->eul);
			if (err < 0) {
				return err;
			}
			err = bno055_vector4_fetch(dev, BNO055_REGISTER_QUA_DATA, &data->qua);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_LIA_DATA, &data->lia);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GRV_DATA, &data->grv);
			if (err < 0) {
				return err;
			}
			err = bno055_calibration_fetch(dev, &data->calib);
			if (err < 0) {
				return err;
			}
			break;

		case BNO055_MODE_NDOF:
			LOG_INF("NDOF fetching..");
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_EUL_DATA, &data->eul);
			if (err < 0) {
				return err;
			}
			err = bno055_vector4_fetch(dev, BNO055_REGISTER_QUA_DATA, &data->qua);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_LIA_DATA, &data->lia);
			if (err < 0) {
				return err;
			}
			err = bno055_vector3_fetch(dev, BNO055_REGISTER_GRV_DATA, &data->grv);
			if (err < 0) {
				return err;
			}
			err = bno055_calibration_fetch(dev, &data->calib);
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

	if (chan == SENSOR_CHAN_ACCEL_X) {
		(val)->val1 = data->acc.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->acc.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_Y) {
		(val)->val1 = data->acc.y / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->acc.y - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_Z) {
		(val)->val1 = data->acc.z / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->acc.z - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		(val)->val1 = data->acc.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->acc.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		(val+1)->val1 = data->acc.y / BNO055_ACCEL_RESOLUTION;
		(val+1)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->acc.y - (val+1)->val1 * BNO055_ACCEL_RESOLUTION);
		(val+2)->val1 = data->acc.z / BNO055_ACCEL_RESOLUTION;
		(val+2)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->acc.z - (val+2)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_GYRO_X) {
		(val)->val1 = data->gyr.x / BNO055_GYRO_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) * (data->gyr.x - (val)->val1 * BNO055_GYRO_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_GYRO_Y) {
		(val)->val1 = data->gyr.y / BNO055_GYRO_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) * (data->gyr.y - (val)->val1 * BNO055_GYRO_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_GYRO_Z) {
		(val)->val1 = data->gyr.z / BNO055_GYRO_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) * (data->gyr.z - (val)->val1 * BNO055_GYRO_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_GYRO_XYZ) {
		(val)->val1 = data->gyr.x / BNO055_GYRO_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) * (data->gyr.x - (val)->val1 * BNO055_GYRO_RESOLUTION);
		(val+1)->val1 = data->gyr.y / BNO055_GYRO_RESOLUTION;
		(val+1)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) * (data->gyr.y - (val+1)->val1 * BNO055_GYRO_RESOLUTION);
		(val+2)->val1 = data->gyr.z / BNO055_GYRO_RESOLUTION;
		(val+2)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) * (data->gyr.z - (val+2)->val1 * BNO055_GYRO_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_MAGN_X) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.x) / BNO055_UTESLA_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) * ((BNO055_UTESLA_TO_GAUSS * data->mag.x) - (val)->val1 * BNO055_UTESLA_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_MAGN_Y) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.y) / BNO055_UTESLA_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) * ((BNO055_UTESLA_TO_GAUSS * data->mag.y) - (val)->val1 * BNO055_UTESLA_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_MAGN_Z) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.z) / BNO055_UTESLA_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) * ((BNO055_UTESLA_TO_GAUSS * data->mag.z) - (val)->val1 * BNO055_UTESLA_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_MAGN_XYZ) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.x) / BNO055_UTESLA_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) * ((BNO055_UTESLA_TO_GAUSS * data->mag.x) - (val)->val1 * BNO055_UTESLA_RESOLUTION);
		(val+1)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.y) / BNO055_UTESLA_RESOLUTION;
		(val+1)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) * ((BNO055_UTESLA_TO_GAUSS * data->mag.y) - (val+1)->val1 * BNO055_UTESLA_RESOLUTION);
		(val+2)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.z) / BNO055_UTESLA_RESOLUTION;
		(val+2)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) * ((BNO055_UTESLA_TO_GAUSS * data->mag.z) - (val+2)->val1 * BNO055_UTESLA_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_Y) {
		(val)->val1 = data->eul.x / BNO055_EULER_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_EULER_RESOLUTION) * (data->eul.x - (val)->val1 * BNO055_EULER_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_R) {
		(val)->val1 = data->eul.y / BNO055_EULER_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_EULER_RESOLUTION) * (data->eul.y - (val)->val1 * BNO055_EULER_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_P) {
		(val)->val1 = data->eul.z / BNO055_EULER_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_EULER_RESOLUTION) * (data->eul.z - (val)->val1 * BNO055_EULER_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_YRP) {
		(val)->val1 = data->eul.x / BNO055_EULER_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_EULER_RESOLUTION) * (data->eul.x - (val)->val1 * BNO055_EULER_RESOLUTION);
		(val+1)->val1 = data->eul.y / BNO055_EULER_RESOLUTION;
		(val+1)->val2 = (1000000 / BNO055_EULER_RESOLUTION) * (data->eul.y - (val+1)->val1 * BNO055_EULER_RESOLUTION);
		(val+2)->val1 = data->eul.z / BNO055_EULER_RESOLUTION;
		(val+2)->val2 = (1000000 / BNO055_EULER_RESOLUTION) * (data->eul.z - (val+2)->val1 * BNO055_EULER_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_W) {
		(val)->val1 = data->qua.w / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.w - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_X) {
		(val)->val1 = data->qua.x / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.x - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_Y) {
		(val)->val1 = data->qua.y / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.y - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_Z) {
		(val)->val1 = data->qua.z / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.z - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_WXYZ) {
		(val)->val1 = data->qua.w / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.w - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		(val+1)->val1 = data->qua.x / BNO055_QUATERNION_RESOLUTION;
		(val+1)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.x - (val+1)->val1 * BNO055_QUATERNION_RESOLUTION);
		(val+2)->val1 = data->qua.y / BNO055_QUATERNION_RESOLUTION;
		(val+2)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.y - (val+2)->val1 * BNO055_QUATERNION_RESOLUTION);
		(val+3)->val1 = data->qua.z / BNO055_QUATERNION_RESOLUTION;
		(val+3)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) * (data->qua.z - (val+3)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_LINEAR_ACCEL_X) {
		(val)->val1 = data->lia.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->lia.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_LINEAR_ACCEL_Y) {
		(val)->val1 = data->lia.y / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->lia.y - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_LINEAR_ACCEL_Z) {
		(val)->val1 = data->lia.z / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->lia.z - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_LINEAR_ACCEL_XYZ) {
		(val)->val1 = data->lia.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->lia.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		(val+1)->val1 = data->lia.y / BNO055_ACCEL_RESOLUTION;
		(val+1)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->lia.y - (val+1)->val1 * BNO055_ACCEL_RESOLUTION);
		(val+2)->val1 = data->lia.z / BNO055_ACCEL_RESOLUTION;
		(val+2)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->lia.z - (val+2)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_GRAVITY_X) {
		(val)->val1 = data->grv.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->grv.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_GRAVITY_Y) {
		(val)->val1 = data->grv.y / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->grv.y - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_GRAVITY_Z) {
		(val)->val1 = data->grv.z / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->grv.z - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_GRAVITY_XYZ) {
		(val)->val1 = data->grv.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->grv.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		(val+1)->val1 = data->grv.y / BNO055_ACCEL_RESOLUTION;
		(val+1)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->grv.y - (val+1)->val1 * BNO055_ACCEL_RESOLUTION);
		(val+2)->val1 = data->grv.z / BNO055_ACCEL_RESOLUTION;
		(val+2)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) * (data->grv.z - (val+2)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_SYS) {
		(val)->val1 = data->calib.sys;
		(val)->val2 = 0;
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_GYR) {
		(val)->val1 = data->calib.gyr;
		(val)->val2 = 0;
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_ACC) {
		(val)->val1 = data->calib.acc;
		(val)->val2 = 0;
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_MAG) {
		(val)->val1 = data->calib.mag;
		(val)->val2 = 0;
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_CALIBRATION_SGAM) {
		(val)->val1 = data->calib.sys;
		(val)->val2 = 0;
		(val+1)->val1 = data->calib.gyr;
		(val+1)->val2 = 0;
		(val+2)->val1 = data->calib.acc;
		(val+2)->val2 = 0;
		(val+3)->val1 = data->calib.mag;
		(val+3)->val2 = 0;
		return 0;
	}
		
	return -ENOTSUP;
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
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, BNO055_PAGE_ZERO);
	if (err < 0) {
		return err;
	}
	data->current_page = BNO055_PAGE_ZERO;
	
	/* Send Reset Command */
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER, BNO055_COMMAND_RESET);
	if (err < 0) {
		return err;
	}
	data->mode = BNO055_MODE_CONFIG;
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
	uint8_t selection = (BNO055_ORIENTATION_WINDOWS << 7)	|
						(BNO055_TEMP_UNIT_CELSIUS << 4)		|
						(BNO055_EULER_UNIT_RADIANS << 2)	|
						(BNO055_GYRO_UNIT_RPS << 1)			|
						(BNO055_ACCEL_UNIT_MS_2 << 0);
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
