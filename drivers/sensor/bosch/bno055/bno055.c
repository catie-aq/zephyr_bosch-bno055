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

#if defined(CONFIG_BNO055_ACC_CUSTOM_CONFIG) || defined(CONFIG_BNO055_MAG_CUSTOM_CONFIG) || defined(CONFIG_BNO055_GYR_CUSTOM_CONFIG)
	if (!fusion) {
		/* Switch to Page 1 */
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, PAGE_ONE);
		if (err < 0) {
			return err;
		}
		data->current_page = PAGE_ONE;

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
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, PAGE_ZERO);
		if (err < 0) {
			return err;
		}
		data->current_page = PAGE_ZERO;
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
					case CONFIG_MODE:
						err = bno055_set_config(dev, CONFIG_MODE, false);
						if (err < 0) return err;
						break;

					case ACC_ONLY:
						err = bno055_set_config(dev, ACC_ONLY, false);
						if (err < 0) return err;
						break;

					case MAG_ONLY:
						err = bno055_set_config(dev, MAG_ONLY, false);
						if (err < 0) return err;
						break;
					
					case GYRO_ONLY:
						err = bno055_set_config(dev, GYRO_ONLY, false);
						if (err < 0) return err;
						break;

					case ACC_MAG:
						err = bno055_set_config(dev, ACC_MAG, false);
						if (err < 0) return err;
						break;

					case ACC_GYRO:
						err = bno055_set_config(dev, ACC_GYRO, false);
						if (err < 0) return err;
						break;

					case MAG_GYRO:
						err = bno055_set_config(dev, MAG_GYRO, false);
						if (err < 0) return err;
						break;

					case ACC_MAG_GYRO:
						err = bno055_set_config(dev, ACC_MAG_GYRO, false);
						if (err < 0) return err;
						break;

					case IMU:
						err = bno055_set_config(dev, IMU, true);
						if (err < 0) return err;
						break;

					case COMPASS:
						err = bno055_set_config(dev, COMPASS, true);
						if (err < 0) return err;
						break;

					case M4G:
						err = bno055_set_config(dev, M4G, true);
						if (err < 0) return err;
						break;

					case NDOF_FMC_OFF:
						err = bno055_set_config(dev, NDOF_FMC_OFF, true);
						if (err < 0) return err;
						break;

					case NDOF:
						err = bno055_set_config(dev, NDOF, true);
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

		case COMPASS:
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

		case M4G:
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

		case NDOF_FMC_OFF:
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

		case NDOF:
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
		(val)->val1 = data->acc.x / AccelerationResolution[data->units.acceleration];
		(val)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->acc.x - (val)->val1 * AccelerationResolution[data->units.acceleration]);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_Y) {
		(val)->val1 = data->acc.y / AccelerationResolution[data->units.acceleration];
		(val)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->acc.y - (val)->val1 * AccelerationResolution[data->units.acceleration]);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_Z) {
		(val)->val1 = data->acc.z / AccelerationResolution[data->units.acceleration];
		(val)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->acc.z - (val)->val1 * AccelerationResolution[data->units.acceleration]);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		(val)->val1 = data->acc.x / AccelerationResolution[data->units.acceleration];
		(val)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->acc.x - (val)->val1 * AccelerationResolution[data->units.acceleration]);
		(val+1)->val1 = data->acc.y / AccelerationResolution[data->units.acceleration];
		(val+1)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->acc.y - (val+1)->val1 * AccelerationResolution[data->units.acceleration]);
		(val+2)->val1 = data->acc.z / AccelerationResolution[data->units.acceleration];
		(val+2)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->acc.z - (val+2)->val1 * AccelerationResolution[data->units.acceleration]);
		return 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_GYRO_XYZ) {
		(val)->val1 = data->gyr.x / RotationResolution[data->units.rotation];
		(val)->val2 = (1000000 / RotationResolution[data->units.rotation]) * (data->gyr.x - (val)->val1 * RotationResolution[data->units.rotation]);
		(val+1)->val1 = data->gyr.y / RotationResolution[data->units.rotation];
		(val+1)->val2 = (1000000 / RotationResolution[data->units.rotation]) * (data->gyr.y - (val+1)->val1 * RotationResolution[data->units.rotation]);
		(val+2)->val1 = data->gyr.z / RotationResolution[data->units.rotation];
		(val+2)->val2 = (1000000 / RotationResolution[data->units.rotation]) * (data->gyr.z - (val+2)->val1 * RotationResolution[data->units.rotation]);
		return 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_MAGN_XYZ) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.x) / uTeslaResolution;
		(val)->val2 = (1000000 / uTeslaResolution) * ((BNO055_UTESLA_TO_GAUSS * data->mag.x) - (val)->val1 * uTeslaResolution);
		(val+1)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.y) / uTeslaResolution;
		(val+1)->val2 = (1000000 / uTeslaResolution) * ((BNO055_UTESLA_TO_GAUSS * data->mag.y) - (val+1)->val1 * uTeslaResolution);
		(val+2)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.z) / uTeslaResolution;
		(val+2)->val2 = (1000000 / uTeslaResolution) * ((BNO055_UTESLA_TO_GAUSS * data->mag.z) - (val+2)->val1 * uTeslaResolution);
		return 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_EULER_YRP) {
		(val)->val1 = data->eul.x / EulerResolution[data->units.euler];
		(val)->val2 = (1000000 / EulerResolution[data->units.euler]) * (data->eul.x - (val)->val1 * EulerResolution[data->units.euler]);
		(val+1)->val1 = data->eul.y / EulerResolution[data->units.euler];
		(val+1)->val2 = (1000000 / EulerResolution[data->units.euler]) * (data->eul.y - (val+1)->val1 * EulerResolution[data->units.euler]);
		(val+2)->val1 = data->eul.z / EulerResolution[data->units.euler];
		(val+2)->val2 = (1000000 / EulerResolution[data->units.euler]) * (data->eul.z - (val+2)->val1 * EulerResolution[data->units.euler]);
		return 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_QUADTERNION_WXYZ) {
		(val)->val1 = data->qua.w / QuaternionResolution;
		(val)->val2 = (1000000 / QuaternionResolution) * (data->qua.w - (val)->val1 * QuaternionResolution);
		(val+1)->val1 = data->qua.x / QuaternionResolution;
		(val+1)->val2 = (1000000 / QuaternionResolution) * (data->qua.x - (val+1)->val1 * QuaternionResolution);
		(val+2)->val1 = data->qua.y / QuaternionResolution;
		(val+2)->val2 = (1000000 / QuaternionResolution) * (data->qua.y - (val+2)->val1 * QuaternionResolution);
		(val+3)->val1 = data->qua.z / QuaternionResolution;
		(val+3)->val2 = (1000000 / QuaternionResolution) * (data->qua.z - (val+3)->val1 * QuaternionResolution);
		return 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_LINEAR_ACCEL_XYZ) {
		(val)->val1 = data->lia.x / AccelerationResolution[data->units.acceleration];
		(val)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->lia.x - (val)->val1 * AccelerationResolution[data->units.acceleration]);
		(val+1)->val1 = data->lia.y / AccelerationResolution[data->units.acceleration];
		(val+1)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->lia.y - (val+1)->val1 * AccelerationResolution[data->units.acceleration]);
		(val+2)->val1 = data->lia.z / AccelerationResolution[data->units.acceleration];
		(val+2)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->lia.z - (val+2)->val1 * AccelerationResolution[data->units.acceleration]);
		return 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_GRAVITY_XYZ) {
		(val)->val1 = data->grv.x / AccelerationResolution[data->units.acceleration];
		(val)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->grv.x - (val)->val1 * AccelerationResolution[data->units.acceleration]);
		(val+1)->val1 = data->grv.y / AccelerationResolution[data->units.acceleration];
		(val+1)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->grv.y - (val+1)->val1 * AccelerationResolution[data->units.acceleration]);
		(val+2)->val1 = data->grv.z / AccelerationResolution[data->units.acceleration];
		(val+2)->val2 = (1000000 / AccelerationResolution[data->units.acceleration]) * (data->grv.z - (val+2)->val1 * AccelerationResolution[data->units.acceleration]);
		return 0;
	}

	if (chan == (enum sensor_channel)SENSOR_CHAN_CALIBRATION_SGAM) {
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
	// /!\ To DO: LOG_INF sensors config
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
