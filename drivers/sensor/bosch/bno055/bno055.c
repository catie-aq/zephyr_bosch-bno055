/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bno055

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "bno055.h"

LOG_MODULE_REGISTER(BNO055, CONFIG_SENSOR_LOG_LEVEL);

struct bno055_config {
	struct i2c_dt_spec i2c_bus;
	bool use_xtal;
	bool deferred;

#if BNO055_USE_IRQ
	const struct gpio_dt_spec irq_gpio;
#endif
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

#if BNO055_USE_IRQ
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t trigger_handler[BNO055_IRQ_SIZE];
	const struct sensor_trigger *trigger[BNO055_IRQ_SIZE];
	struct k_work cb_work;

	struct sensor_value acc_am;
#endif
};

static int bno055_set_page(const struct device *dev, enum PageId page)
{
	const struct bno055_config *config = dev->config;
	struct bno055_data *data = dev->data;
	uint8_t reg;
	int err;

	LOG_DBG("FUNC PAGE[%d]", page);
	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, &reg);
	if (err < 0) {
		return err;
	}

	if (data->current_page != (reg & BNO055_PAGE_ID_MASK)) {
		LOG_WRN("[W] I2C page register issue [%d]|[%d]!!", data->current_page,
			reg & BNO055_PAGE_ID_MASK);
		data->current_page = reg & BNO055_PAGE_ID_MASK;
	}

	if ((reg & BNO055_PAGE_ID_MASK) == page) {
		LOG_DBG("I2C page register already good!!");
		return 0;
	}

	// Write PAGE
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, page);
	if (err < 0) {
		return err;
	}

	// Read PAGE
	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_PAGE_ID, &reg);
	if (err < 0) {
		return err;
	}

	if ((reg & BNO055_PAGE_ID_MASK) != page) {
		LOG_ERR("[E] I2C communication compromised [%d]|[%d]!!", reg & BNO055_PAGE_ID_MASK,
			page);
		return -1;
	}

	data->current_page = reg & BNO055_PAGE_ID_MASK;
	LOG_DBG("FUNC PAGE[%d]", page);
	return 0;
}

static int bno055_set_config(const struct device *dev, enum OperatingMode mode, bool fusion)
{
	const struct bno055_config *config = dev->config;
	struct bno055_data *data = dev->data;
	uint8_t reg;
	int err;

	LOG_DBG("FUNC MODE[%d]", mode);

	/* Switch to Page 0 */
	err = bno055_set_page(dev, BNO055_PAGE_ZERO);
	if (err < 0) {
		return err;
	}

	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE, &reg);
	if (err < 0) {
		return err;
	}

	if (data->mode != (reg & BNO055_OPERATION_MODE_MASK)) {
		LOG_WRN("[WC_1] I2C mode register issue [%d]|[%d]!!", data->mode,
			reg & BNO055_OPERATION_MODE_MASK);
		data->mode = reg & BNO055_OPERATION_MODE_MASK;
	}

	if ((reg & BNO055_OPERATION_MODE_MASK) != BNO055_MODE_CONFIG) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE,
					    BNO055_MODE_CONFIG);
		if (err < 0) {
			return err;
		}
		LOG_DBG("MODE[%d]", BNO055_MODE_CONFIG);
		k_sleep(K_MSEC(BNO055_TIMING_SWITCH_FROM_ANY));
	}

	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE, &reg);
	if (err < 0) {
		return err;
	}

	if ((reg & BNO055_OPERATION_MODE_MASK) != BNO055_MODE_CONFIG) {
		LOG_ERR("[EC_1] I2C communication compromised [%d]|[%d]!!",
			reg & BNO055_OPERATION_MODE_MASK, BNO055_MODE_CONFIG);
		return -1;
	}
	data->mode = reg & BNO055_OPERATION_MODE_MASK;

	if (mode == BNO055_MODE_CONFIG) {
		LOG_DBG("I2C mode register already good!!");
		return 0;
	}

#if defined(CONFIG_BNO055_ACC_CUSTOM_CONFIG) || defined(CONFIG_BNO055_MAG_CUSTOM_CONFIG) ||        \
	defined(CONFIG_BNO055_GYR_CUSTOM_CONFIG)
	if (!fusion) {
		/* Switch to Page 1 */
		bno055_set_page(dev, BNO055_PAGE_ONE);

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
		err = bno055_set_page(dev, BNO055_PAGE_ZERO);
		if (err < 0) {
			return err;
		}
	}
#endif

	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE, mode);
	if (err < 0) {
		return err;
	}
	k_sleep(K_MSEC(
		33 *
		BNO055_TIMING_SWITCH_FROM_CONFIG)); // /!\ not datasheet confrom WRONG DATASHEET

	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_OPERATION_MODE, &reg);
	if (err < 0) {
		return err;
	}

	if ((reg & BNO055_PAGE_ID_MASK) != mode) {
		LOG_ERR("[EC_2] I2C communication compromised [%d]|[%d]!!",
			reg & BNO055_OPERATION_MODE_MASK, mode);
		return -1;
	}

	data->mode = reg & BNO055_OPERATION_MODE_MASK;
	LOG_DBG("FUNC MODE[%d]", mode);
	return 0;
}

static int bno055_set_power(const struct device *dev, enum PowerMode power)
{
	const struct bno055_config *config = dev->config;
	struct bno055_data *data = dev->data;
	uint8_t reg;
	int err;

	LOG_DBG("FUNC POWER[%d]", power);

	enum OperatingMode mode = data->mode;
	err = bno055_set_config(dev, BNO055_MODE_CONFIG, false);
	if (err < 0) {
		return err;
	}

	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_POWER_MODE, &reg);
	if (err < 0) {
		return err;
	}

	if (data->power != (reg & BNO055_POWER_MODE_MASK)) {
		LOG_WRN("[WP_1] I2C mode register issue [%d]|[%d]!!", data->power,
			reg & BNO055_POWER_MODE_MASK);
		data->power = reg & BNO055_POWER_MODE_MASK;
	}

	if ((reg & BNO055_POWER_MODE_MASK) == power) {
		LOG_DBG("I2C power register already good!!");
	} else {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_POWER_MODE, power);
		if (err < 0) {
			return err;
		}

		err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_POWER_MODE, &power);
		if (err < 0) {
			return err;
		}

		if ((reg & BNO055_POWER_MODE_MASK) != mode) {
			LOG_ERR("[EP_1] I2C communication compromised [%d]|[%d]!!",
				reg & BNO055_POWER_MODE_MASK, mode);
			return -1;
		}
		data->power = reg & BNO055_POWER_MODE_MASK;
	}

	err = bno055_set_config(dev, mode, mode < BNO055_MODE_IMU ? false : true);
	if (err < 0) {
		return err;
	}

	LOG_DBG("FUNC POWER[%d]", power);
	return 0;
}

static int bno055_set_attribut(const struct device *dev, uint8_t reg, uint8_t mask, uint8_t shift,
			       uint8_t val)
{
	const struct bno055_config *config = dev->config;
	struct bno055_data *data = dev->data;
	uint8_t res;
	int err;

	LOG_DBG("FUNC ATTR[%d][%d]", reg, val);

	enum OperatingMode mode = data->mode;
	err = bno055_set_config(dev, BNO055_MODE_CONFIG, false);
	if (err < 0) {
		return err;
	}

	/* Switch to Page 1 */
	bno055_set_page(dev, BNO055_PAGE_ONE);

	err = i2c_reg_read_byte_dt(&config->i2c_bus, reg, &res);
	if (err < 0) {
		return err;
	}

	res &= ~mask;
	res |= (val << shift) & mask;
	err = i2c_reg_write_byte_dt(&config->i2c_bus, reg, res);
	if (err < 0) {
		return err;
	}

	err = i2c_reg_read_byte_dt(&config->i2c_bus, reg, &res);
	if (err < 0) {
		return err;
	}

	if ((res & mask) != (val << shift)) {
		LOG_ERR("[EP_1] I2C communication compromised [%d]|[%d]!!", res & mask,
			val << shift);
		return -1;
	}

	err = bno055_set_config(dev, mode, mode < BNO055_MODE_IMU ? false : true);
	if (err < 0) {
		return err;
	}

	LOG_DBG("FUNC ATTR[%d][%d]", reg, (res & mask) >> shift);
	return 0;
}

static int bno055_vector3_fetch(const struct device *dev, const uint8_t data_register,
				struct vector3_data *data)
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

static int bno055_vector4_fetch(const struct device *dev, const uint8_t data_register,
				struct vector4_data *data)
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

	int err = i2c_burst_read_dt(&config->i2c_bus, BNO055_REGISTER_CALIBRATION_STATUS, regs,
				    sizeof(regs));
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

	switch (chan) {
	case SENSOR_CHAN_ALL:
		if (attr == SENSOR_ATTR_CONFIGURATION) {
			LOG_INF("SET MODE[%d]", val->val1);
			switch (val->val1) {
			case BNO055_MODE_CONFIG:
				LOG_DBG("MODE BNO055_MODE_CONFIG");
				err = bno055_set_config(dev, BNO055_MODE_CONFIG, false);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_ACC_ONLY:
				LOG_DBG("MODE BNO055_MODE_ACC_ONLY");
				err = bno055_set_config(dev, BNO055_MODE_ACC_ONLY, false);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_MAG_ONLY:
				LOG_DBG("MODE BNO055_MODE_MAG_ONLY");
				err = bno055_set_config(dev, BNO055_MODE_MAG_ONLY, false);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_GYRO_ONLY:
				LOG_DBG("MODE BNO055_MODE_GYRO_ONLY");
				err = bno055_set_config(dev, BNO055_MODE_GYRO_ONLY, false);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_ACC_MAG:
				LOG_DBG("MODE BNO055_MODE_ACC_MAG");
				err = bno055_set_config(dev, BNO055_MODE_ACC_MAG, false);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_ACC_GYRO:
				LOG_DBG("MODE BNO055_MODE_ACC_GYRO");
				err = bno055_set_config(dev, BNO055_MODE_ACC_GYRO, false);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_MAG_GYRO:
				LOG_DBG("MODE BNO055_MODE_MAG_GYRO");
				err = bno055_set_config(dev, BNO055_MODE_MAG_GYRO, false);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_ACC_MAG_GYRO:
				LOG_DBG("MODE BNO055_MODE_ACC_MAG_GYRO");
				err = bno055_set_config(dev, BNO055_MODE_ACC_MAG_GYRO, false);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_IMU:
				LOG_DBG("MODE BNO055_MODE_IMU");
				err = bno055_set_config(dev, BNO055_MODE_IMU, true);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_COMPASS:
				LOG_DBG("MODE BNO055_MODE_COMPASS");
				err = bno055_set_config(dev, BNO055_MODE_COMPASS, true);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_M4G:
				LOG_DBG("MODE BNO055_MODE_M4G");
				err = bno055_set_config(dev, BNO055_MODE_M4G, true);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_NDOF_FMC_OFF:
				LOG_DBG("MODE BNO055_MODE_NDOF_FMC_OFF");
				err = bno055_set_config(dev, BNO055_MODE_NDOF_FMC_OFF, true);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_MODE_NDOF:
				LOG_DBG("MODE BNO055_MODE_NDOF");
				err = bno055_set_config(dev, BNO055_MODE_NDOF, true);
				if (err < 0) {
					return err;
				}
				break;

			default:
				return -EINVAL;
			}
		} else if (attr == (enum sensor_attribute)BNO055_SENSOR_ATTR_POWER_MODE) {
			LOG_INF("SET POWER[%d]", val->val1);
			switch (val->val1) {
			case BNO055_POWER_NORMAL:
				LOG_DBG("POWER BNO055_POWER_NORMAL");
				err = bno055_set_power(dev, BNO055_POWER_NORMAL);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_POWER_LOW_POWER:
				LOG_DBG("POWER BNO055_POWER_LOW_POWER");
				err = bno055_set_power(dev, BNO055_POWER_LOW_POWER);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_POWER_SUSPEND:
				LOG_DBG("POWER BNO055_POWER_SUSPEND");
				err = bno055_set_power(dev, BNO055_POWER_SUSPEND);
				if (err < 0) {
					return err;
				}
				break;

			case BNO055_POWER_INVALID:
				LOG_DBG("POWER BNO055_POWER_INVALID");
				err = bno055_set_power(dev, BNO055_POWER_INVALID);
				if (err < 0) {
					return err;
				}
				break;

			default:
				return -EINVAL;
			}
		}
		break;

	case SENSOR_CHAN_ACCEL_XYZ:
		LOG_INF("SET ACC ATTR[%d][%d]", attr, val->val1);
		switch (attr) {
		case SENSOR_ATTR_SLOPE_TH:
			if (val->val1 == BNO055_ACC_THRESHOLD_AM) {
				LOG_DBG("ACC ATTR AM THRESHOLD");
				err = bno055_set_attribut(dev,
							  BNO055_REGISTER_ACC_ANY_MOTION_THRESHOLD,
							  BNO055_IRQ_ACC_MASK_THRESHOLD,
							  BNO055_IRQ_ACC_SHIFT_AM, val->val2);
				if (err < 0) {
					return err;
				}
			} else if (val->val1 == BNO055_ACC_THRESHOLD_NM) {
				LOG_DBG("ACC ATTR NM THRESHOLD");
				err = bno055_set_attribut(dev,
							  BNO055_REGISTER_ACC_NO_MOTION_THRESHOLD,
							  BNO055_IRQ_ACC_MASK_SNM_THRESHOLD,
							  BNO055_IRQ_ACC_SHIFT_SNM, val->val2);
				if (err < 0) {
					return err;
				}
			} else if (val->val1 == BNO055_ACC_THRESHOLD_HG) {
				LOG_DBG("ACC ATTR HG THRESHOLD");
				err = bno055_set_attribut(
					dev, BNO055_REGISTER_ACC_HIGH_GRAVITY_THRESHOLD,
					BNO055_IRQ_ACC_MASK_HG_THRESHOLD, BNO055_IRQ_ACC_NO_SHIFT,
					val->val2);
				if (err < 0) {
					return err;
				}
			} else {
				return -ENOTSUP;
			}
			break;

		case SENSOR_ATTR_SLOPE_DUR:
			if (val->val1 == BNO055_ACC_DURATION_AM) {
				LOG_DBG("ACC ATTR AM DURATION");
				err = bno055_set_attribut(dev, BNO055_REGISTER_ACC_INT_SETTINGS,
							  BNO055_IRQ_ACC_MASK_AM_DURATION,
							  BNO055_IRQ_ACC_SHIFT_AM, val->val2);
				if (err < 0) {
					return err;
				}
			} else if (val->val1 == BNO055_ACC_DURATION_NM) {
				LOG_DBG("ACC ATTR NM DURATION");
				uint8_t snm = (val->val2 >> BNO055_IRQ_ACC_SNM_SHIFT);
				uint32_t duration = val->val2 & (~BNO055_IRQ_ACC_SN_MOTION_NO);
				uint8_t value = duration & (BNO055_IRQ_ACC_MASK_SLOW_DURATION >>
							    BNO055_IRQ_ACC_SHIFT_SNM);

				if (snm == BNO055_IRQ_ACC_SN_MOTION_NO) {
					value = (duration > BNO055_ACC_SN_DURATION_80_SECONDS)
							? (0x01 << 5)
							: 0x00;
					if (duration > BNO055_ACC_SN_DURATION_80_SECONDS) {
						value |= ((duration - 88) >>
							  3); // duration = (reg * 8) + 88
					} else {
						value |= (duration >
							  BNO055_ACC_SN_DURATION_20_SECONDS)
								 ? (0x01 << 4)
								 : 0x00;
						if (duration > BNO055_ACC_SN_DURATION_20_SECONDS) {
							value |= ((duration - 20) >>
								  2); // duration = (reg * 4) + 20
						} else {
							value |= ((duration - 1) >>
								  0); // duration = (reg * 1) + 1
						}
					}
				}

				err = bno055_set_attribut(
					dev, BNO055_REGISTER_ACC_NO_MOTION_SET,
					(snm == BNO055_IRQ_ACC_SN_MOTION_NO)
						? BNO055_IRQ_ACC_MASK_SNM_DURATION
						: BNO055_IRQ_ACC_MASK_SLOW_DURATION,
					BNO055_IRQ_ACC_SHIFT_SNM, value);
				if (err < 0) {
					return err;
				}
				err = bno055_set_attribut(dev, BNO055_REGISTER_ACC_NO_MOTION_SET,
							  BNO055_IRQ_ACC_MASK_SNM_SET,
							  BNO055_IRQ_ACC_NO_SHIFT, snm);
				if (err < 0) {
					return err;
				}
			} else if (val->val1 == BNO055_ACC_DURATION_HG) {
				LOG_DBG("ACC ATTR HG DURATION");
				err = bno055_set_attribut(dev,
							  BNO055_REGISTER_ACC_HIGH_GRAVITY_DURATION,
							  BNO055_IRQ_ACC_MASK_HG_DURATION,
							  BNO055_IRQ_ACC_NO_SHIFT, val->val2);
				if (err < 0) {
					return err;
				}
			} else {
				return -ENOTSUP;
			}
			break;

		default:
			return -ENOTSUP;
		}
		break;

	case SENSOR_CHAN_GYRO_XYZ:
		LOG_INF("SET GYRO ATTR[%d][%d]", attr, val->val1);
		switch (attr) {
		case SENSOR_ATTR_SLOPE_TH:
			LOG_DBG("ACC GYRO AM THRESHOLD");
			err = bno055_set_attribut(dev, BNO055_REGISTER_GYR_AM_THRESHOLD,
						  BNO055_IRQ_GYR_MASK_AM_THRESHOLD,
						  BNO055_IRQ_GYR_SHIFT_AM_THRESHOLD, val->val1);
			if (err < 0) {
				return err;
			}
			err = bno055_set_attribut(dev, BNO055_REGISTER_GYR_INT_SETTINGS,
						  BNO055_IRQ_GYR_SETTINGS_AM_FILT,
						  BNO055_IRQ_GYR_SHIFT_AM_FILT, val->val2);
			if (err < 0) {
				return err;
			}
			break;

		case SENSOR_ATTR_SLOPE_DUR:
			LOG_DBG("ACC ATTR AM DURATION");
			err = bno055_set_attribut(dev, BNO055_REGISTER_GYR_AM_SET,
						  BNO055_IRQ_GYR_MASK_AM_AWAKE_DURATION,
						  BNO055_IRQ_GYR_SHIFT_AM_AWAKE_DURATION,
						  val->val1);
			if (err < 0) {
				return err;
			}
			err = bno055_set_attribut(dev, BNO055_REGISTER_GYR_AM_SET,
						  BNO055_IRQ_GYR_MASK_AM_SAMPLES,
						  BNO055_IRQ_GYR_SHIFT_AM_SAMPLES, val->val1);
			if (err < 0) {
				return err;
			}
			break;

		default:
			return -ENOTSUP;
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
	int err;

	/* Switch to Page 0 */
	bno055_set_page(dev, BNO055_PAGE_ZERO);

	switch (data->mode) {
	case BNO055_MODE_CONFIG:
		LOG_WRN("CONFIG Mode no sample");
		break;

	case BNO055_MODE_ACC_ONLY:
		LOG_DBG("ACC fetching..");
		err = bno055_vector3_fetch(dev, BNO055_REGISTER_ACC_DATA, &data->acc);
		if (err < 0) {
			return err;
		}
		break;

	case BNO055_MODE_MAG_ONLY:
		LOG_DBG("MAG fetching..");
		err = bno055_vector3_fetch(dev, BNO055_REGISTER_MAG_DATA, &data->mag);
		if (err < 0) {
			return err;
		}
		break;

	case BNO055_MODE_GYRO_ONLY:
		LOG_DBG("GYR fetching..");
		err = bno055_vector3_fetch(dev, BNO055_REGISTER_GYR_DATA, &data->gyr);
		if (err < 0) {
			return err;
		}
		break;

	case BNO055_MODE_ACC_MAG:
		LOG_DBG("ACC_MAG fetching..");
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
		LOG_DBG("ACC_GYRO fetching..");
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
		LOG_DBG("MAG_GYRO fetching..");
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
		LOG_DBG("ACC_MAG_GYRO fetching..");
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
		LOG_DBG("IMU fetching..");
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
		LOG_DBG("COMPASS fetching..");
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
		LOG_DBG("M4G fetching..");
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
		LOG_DBG("NDOF_FMC_OFF fetching..");
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
		LOG_DBG("NDOF fetching..");
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
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->acc.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_Y) {
		(val)->val1 = data->acc.y / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->acc.y - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_Z) {
		(val)->val1 = data->acc.z / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->acc.z - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_ACCEL_XYZ) {
		(val)->val1 = data->acc.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->acc.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		(val + 1)->val1 = data->acc.y / BNO055_ACCEL_RESOLUTION;
		(val + 1)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
				  (data->acc.y - (val + 1)->val1 * BNO055_ACCEL_RESOLUTION);
		(val + 2)->val1 = data->acc.z / BNO055_ACCEL_RESOLUTION;
		(val + 2)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
				  (data->acc.z - (val + 2)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_GYRO_X) {
		(val)->val1 = data->gyr.x / BNO055_GYRO_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) *
			      (data->gyr.x - (val)->val1 * BNO055_GYRO_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_GYRO_Y) {
		(val)->val1 = data->gyr.y / BNO055_GYRO_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) *
			      (data->gyr.y - (val)->val1 * BNO055_GYRO_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_GYRO_Z) {
		(val)->val1 = data->gyr.z / BNO055_GYRO_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) *
			      (data->gyr.z - (val)->val1 * BNO055_GYRO_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_GYRO_XYZ) {
		(val)->val1 = data->gyr.x / BNO055_GYRO_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) *
			      (data->gyr.x - (val)->val1 * BNO055_GYRO_RESOLUTION);
		(val + 1)->val1 = data->gyr.y / BNO055_GYRO_RESOLUTION;
		(val + 1)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) *
				  (data->gyr.y - (val + 1)->val1 * BNO055_GYRO_RESOLUTION);
		(val + 2)->val1 = data->gyr.z / BNO055_GYRO_RESOLUTION;
		(val + 2)->val2 = (1000000 / BNO055_GYRO_RESOLUTION) *
				  (data->gyr.z - (val + 2)->val1 * BNO055_GYRO_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_MAGN_X) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.x) / BNO055_UTESLA_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) *
			      ((BNO055_UTESLA_TO_GAUSS * data->mag.x) -
			       (val)->val1 * BNO055_UTESLA_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_MAGN_Y) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.y) / BNO055_UTESLA_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) *
			      ((BNO055_UTESLA_TO_GAUSS * data->mag.y) -
			       (val)->val1 * BNO055_UTESLA_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_MAGN_Z) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.z) / BNO055_UTESLA_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) *
			      ((BNO055_UTESLA_TO_GAUSS * data->mag.z) -
			       (val)->val1 * BNO055_UTESLA_RESOLUTION);
		return 0;
	}

	if (chan == SENSOR_CHAN_MAGN_XYZ) {
		(val)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.x) / BNO055_UTESLA_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) *
			      ((BNO055_UTESLA_TO_GAUSS * data->mag.x) -
			       (val)->val1 * BNO055_UTESLA_RESOLUTION);
		(val + 1)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.y) / BNO055_UTESLA_RESOLUTION;
		(val + 1)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) *
				  ((BNO055_UTESLA_TO_GAUSS * data->mag.y) -
				   (val + 1)->val1 * BNO055_UTESLA_RESOLUTION);
		(val + 2)->val1 = (BNO055_UTESLA_TO_GAUSS * data->mag.z) / BNO055_UTESLA_RESOLUTION;
		(val + 2)->val2 = (1000000 / BNO055_UTESLA_RESOLUTION) *
				  ((BNO055_UTESLA_TO_GAUSS * data->mag.z) -
				   (val + 2)->val1 * BNO055_UTESLA_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_Y) {
		(val)->val1 = data->eul.x / BNO055_EULER_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_EULER_RESOLUTION) *
			      (data->eul.x - (val)->val1 * BNO055_EULER_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_R) {
		(val)->val1 = data->eul.y / BNO055_EULER_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_EULER_RESOLUTION) *
			      (data->eul.y - (val)->val1 * BNO055_EULER_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_P) {
		(val)->val1 = data->eul.z / BNO055_EULER_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_EULER_RESOLUTION) *
			      (data->eul.z - (val)->val1 * BNO055_EULER_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_EULER_YRP) {
		(val)->val1 = data->eul.x / BNO055_EULER_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_EULER_RESOLUTION) *
			      (data->eul.x - (val)->val1 * BNO055_EULER_RESOLUTION);
		(val + 1)->val1 = data->eul.y / BNO055_EULER_RESOLUTION;
		(val + 1)->val2 = (1000000 / BNO055_EULER_RESOLUTION) *
				  (data->eul.y - (val + 1)->val1 * BNO055_EULER_RESOLUTION);
		(val + 2)->val1 = data->eul.z / BNO055_EULER_RESOLUTION;
		(val + 2)->val2 = (1000000 / BNO055_EULER_RESOLUTION) *
				  (data->eul.z - (val + 2)->val1 * BNO055_EULER_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_W) {
		(val)->val1 = data->qua.w / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) *
			      (data->qua.w - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_X) {
		(val)->val1 = data->qua.x / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) *
			      (data->qua.x - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_Y) {
		(val)->val1 = data->qua.y / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) *
			      (data->qua.y - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_Z) {
		(val)->val1 = data->qua.z / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) *
			      (data->qua.z - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_QUATERNION_WXYZ) {
		(val)->val1 = data->qua.w / BNO055_QUATERNION_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) *
			      (data->qua.w - (val)->val1 * BNO055_QUATERNION_RESOLUTION);
		(val + 1)->val1 = data->qua.x / BNO055_QUATERNION_RESOLUTION;
		(val + 1)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) *
				  (data->qua.x - (val + 1)->val1 * BNO055_QUATERNION_RESOLUTION);
		(val + 2)->val1 = data->qua.y / BNO055_QUATERNION_RESOLUTION;
		(val + 2)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) *
				  (data->qua.y - (val + 2)->val1 * BNO055_QUATERNION_RESOLUTION);
		(val + 3)->val1 = data->qua.z / BNO055_QUATERNION_RESOLUTION;
		(val + 3)->val2 = (1000000 / BNO055_QUATERNION_RESOLUTION) *
				  (data->qua.z - (val + 3)->val1 * BNO055_QUATERNION_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_LINEAR_ACCEL_X) {
		(val)->val1 = data->lia.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->lia.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_LINEAR_ACCEL_Y) {
		(val)->val1 = data->lia.y / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->lia.y - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_LINEAR_ACCEL_Z) {
		(val)->val1 = data->lia.z / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->lia.z - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_LINEAR_ACCEL_XYZ) {
		(val)->val1 = data->lia.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->lia.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		(val + 1)->val1 = data->lia.y / BNO055_ACCEL_RESOLUTION;
		(val + 1)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
				  (data->lia.y - (val + 1)->val1 * BNO055_ACCEL_RESOLUTION);
		(val + 2)->val1 = data->lia.z / BNO055_ACCEL_RESOLUTION;
		(val + 2)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
				  (data->lia.z - (val + 2)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_GRAVITY_X) {
		(val)->val1 = data->grv.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->grv.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_GRAVITY_Y) {
		(val)->val1 = data->grv.y / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->grv.y - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_GRAVITY_Z) {
		(val)->val1 = data->grv.z / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->grv.z - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		return 0;
	}

	if (chan == (enum sensor_channel)BNO055_SENSOR_CHAN_GRAVITY_XYZ) {
		(val)->val1 = data->grv.x / BNO055_ACCEL_RESOLUTION;
		(val)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
			      (data->grv.x - (val)->val1 * BNO055_ACCEL_RESOLUTION);
		(val + 1)->val1 = data->grv.y / BNO055_ACCEL_RESOLUTION;
		(val + 1)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
				  (data->grv.y - (val + 1)->val1 * BNO055_ACCEL_RESOLUTION);
		(val + 2)->val1 = data->grv.z / BNO055_ACCEL_RESOLUTION;
		(val + 2)->val2 = (1000000 / BNO055_ACCEL_RESOLUTION) *
				  (data->grv.z - (val + 2)->val1 * BNO055_ACCEL_RESOLUTION);
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
		(val + 1)->val1 = data->calib.gyr;
		(val + 1)->val2 = 0;
		(val + 2)->val1 = data->calib.acc;
		(val + 2)->val2 = 0;
		(val + 3)->val1 = data->calib.mag;
		(val + 3)->val2 = 0;
		return 0;
	}

	return -ENOTSUP;
}

#if BNO055_USE_IRQ
static void bno055_gpio_callback_handler(const struct device *p_port, struct gpio_callback *p_cb,
					 uint32_t pins)
{
	ARG_UNUSED(p_port);
	ARG_UNUSED(pins);
	LOG_DBG("Process GPIO callback!!");

	struct bno055_data *data = CONTAINER_OF(p_cb, struct bno055_data, gpio_cb);

	k_work_submit(&data->cb_work); // Using work queue to exit isr context
}

static void bno055_work_cb(struct k_work *p_work)
{
	struct bno055_data *data = CONTAINER_OF(p_work, struct bno055_data, cb_work);
	const struct bno055_config *config = data->dev->config;
	uint8_t reg;
	int err;

	LOG_DBG("Process Trigger worker from interrupt");

	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_IRQ_STATUS, &reg);
	if (err < 0) {
		LOG_ERR("Trigger worker I2C read FLAGS error");
	}

	if (reg & BNO055_IRQ_MASK_ACC_BSX_DRDY) {
		if (data->trigger_handler[BNO055_IRQ_ACC_BSX_DRDY]) {
			LOG_DBG("Calling ACC_BSX_DRDY callback");
			data->trigger_handler[BNO055_IRQ_ACC_BSX_DRDY](
				data->dev, data->trigger[BNO055_IRQ_ACC_BSX_DRDY]);
		}
	}

	if (reg & BNO055_IRQ_MASK_MAG_DRDY) {
		if (data->trigger_handler[BNO055_IRQ_MAG_DRDY]) {
			LOG_DBG("Calling MAG_DRDY callback");
			data->trigger_handler[BNO055_IRQ_MAG_DRDY](
				data->dev, data->trigger[BNO055_IRQ_MAG_DRDY]);
		}
	}

	if (reg & BNO055_IRQ_MASK_GYR_AM) {
		if (data->trigger_handler[BNO055_IRQ_GYR_AM]) {
			LOG_DBG("Calling GYR_AM callback");
			data->trigger_handler[BNO055_IRQ_GYR_AM](data->dev,
								 data->trigger[BNO055_IRQ_GYR_AM]);
		}
	}

	if (reg & BNO055_IRQ_MASK_GYR_HIGH_RATE) {
		if (data->trigger_handler[BNO055_IRQ_GYR_HIGH_RATE]) {
			LOG_DBG("Calling GYR_HIGH_RATE callback");
			data->trigger_handler[BNO055_IRQ_GYR_HIGH_RATE](
				data->dev, data->trigger[BNO055_IRQ_GYR_HIGH_RATE]);
		}
	}

	if (reg & BNO055_IRQ_MASK_GYR_DRDY) {
		if (data->trigger_handler[BNO055_IRQ_GYR_DRDY]) {
			LOG_DBG("Calling GYR_DRDY callback");
			data->trigger_handler[BNO055_IRQ_GYR_DRDY](
				data->dev, data->trigger[BNO055_IRQ_GYR_DRDY]);
		}
	}

	if (reg & BNO055_IRQ_MASK_ACC_HIGH_G) {
		if (data->trigger_handler[BNO055_IRQ_ACC_HIGH_G]) {
			LOG_DBG("Calling ACC_HIGH_G callback");
			data->trigger_handler[BNO055_IRQ_ACC_HIGH_G](
				data->dev, data->trigger[BNO055_IRQ_ACC_HIGH_G]);
		}
	}

	if (reg & BNO055_IRQ_MASK_ACC_AM) {
		if (data->trigger_handler[BNO055_IRQ_ACC_AN_MOTION]) {
			LOG_DBG("Calling ACC_AM callback");
			data->trigger_handler[BNO055_IRQ_ACC_AN_MOTION](
				data->dev, data->trigger[BNO055_IRQ_ACC_AN_MOTION]);
		}
	}

	if (reg & BNO055_IRQ_MASK_ACC_NM) {
		if (data->trigger_handler[BNO055_IRQ_ACC_AN_MOTION]) {
			LOG_DBG("Calling ACC_NM callback");
			data->trigger_handler[BNO055_IRQ_ACC_AN_MOTION](
				data->dev, data->trigger[BNO055_IRQ_ACC_AN_MOTION]);
		}
	}

	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER, &reg);
	if (err < 0) {
		LOG_ERR("Trigger worker I2C read SYS_TRIG error");
	}

	reg |= BNO055_RESET_INT;
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER, reg);
	if (err < 0) {
		LOG_ERR("Trigger worker I2C write SYS_TRIG error");
	}
}

static int bno055_trigger_configuation(const struct device *dev, const struct sensor_trigger *trig,
				       uint8_t irq, uint8_t mask, bool enable)
{
	const struct bno055_config *config = dev->config;
	struct bno055_data *data = dev->data;
	int err;

	LOG_DBG("FUNC TRIGGER[%d][%d]", mask, enable);
	enum OperatingMode mode = data->mode;
	err = bno055_set_config(dev, BNO055_MODE_CONFIG, false);
	if (err < 0) {
		return err;
	}

	/* Switch to Page 1 */
	bno055_set_page(dev, BNO055_PAGE_ONE);

	uint8_t reg[2];
	if ((trig->type == SENSOR_TRIG_DELTA) || (trig->type == SENSOR_TRIG_STATIONARY) ||
	    (trig->type == (enum sensor_trigger_type)BNO055_SENSOR_TRIG_HIGH_G)) {
		uint8_t i2c_reg = BNO055_REGISTER_CHIP_ID;
		uint8_t reg_mask = BNO055_REGISTER_CHIP_ID;
		if ((trig->chan == SENSOR_CHAN_ACCEL_XYZ) || (trig->chan == SENSOR_CHAN_ACCEL_X) ||
		    (trig->chan == SENSOR_CHAN_ACCEL_Y) || (trig->chan == SENSOR_CHAN_ACCEL_Z)) {
			i2c_reg = BNO055_REGISTER_ACC_INT_SETTINGS;
			reg_mask = BNO055_IRQ_ACC_MASK_AN_MOTION_AXIS;
			if (trig->type == (enum sensor_trigger_type)BNO055_SENSOR_TRIG_HIGH_G) {
				reg_mask = BNO055_IRQ_ACC_MASK_HG_AXIS;
			}
		} else if ((trig->chan == SENSOR_CHAN_GYRO_XYZ) ||
			   (trig->chan == SENSOR_CHAN_GYRO_X) ||
			   (trig->chan == SENSOR_CHAN_GYRO_Y) ||
			   (trig->chan == SENSOR_CHAN_GYRO_Z)) {
			i2c_reg = BNO055_REGISTER_GYR_INT_SETTINGS;
			reg_mask = BNO055_IRQ_GYR_MASK_AN_MOTION_AXIS;
		}

		err = i2c_reg_read_byte_dt(&config->i2c_bus, i2c_reg, &reg[0]);
		if (err < 0) {
			return err;
		}

		if (i2c_reg == BNO055_REGISTER_CHIP_ID) {
			return -1;
		}
		reg[0] &= ~reg_mask;
		reg[0] |= mask;

		err = i2c_reg_write_byte_dt(&config->i2c_bus, i2c_reg, reg[0]);
		if (err < 0) {
			return err;
		}
	}

	err = i2c_burst_read_dt(&config->i2c_bus, BNO055_REGISTER_INT_MASK, reg, sizeof(reg));
	if (err < 0) {
		return err;
	}
	LOG_DBG("MASK[%d] | ENABLE[%d]", reg[0], reg[1]);

	if (enable) {
		LOG_DBG("TRIGGER %d Enable!!", irq);
		reg[0] |= irq;
		if (irq & BNO055_IRQ_MASK_ACC_AM) {
			reg[1] &= ~BNO055_IRQ_MASK_ACC_NM;
		} else if (irq & BNO055_IRQ_MASK_ACC_NM) {
			reg[1] &= ~BNO055_IRQ_MASK_ACC_AM;
		}
		reg[1] |= irq;
		LOG_DBG("TARGET[%d][%d]", reg[0], reg[1]);
		err = i2c_burst_write_dt(&config->i2c_bus, BNO055_REGISTER_INT_MASK, reg,
					 sizeof(reg));
		if (err < 0) {
			return err;
		}
	} else {
		LOG_DBG("TRIGGER %d Disable!!", irq);
		reg[0] &= ~irq;
		reg[1] &= ~irq;
		err = i2c_burst_write_dt(&config->i2c_bus, BNO055_REGISTER_INT_MASK, reg,
					 sizeof(reg));
		if (err < 0) {
			return err;
		}
	}

	err = i2c_burst_read_dt(&config->i2c_bus, BNO055_REGISTER_INT_MASK, reg, sizeof(reg));
	if (err < 0) {
		return err;
	}
	LOG_DBG("MASK[%d] | ENABLE[%d]", reg[0], reg[1]);

	err = bno055_set_config(dev, mode, mode < BNO055_MODE_IMU ? false : true);
	if (err < 0) {
		return err;
	}

	LOG_DBG("FUNC TRIGGER[%d][%d]", reg[0], reg[1]);
	return 0;
}

static int bno055_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			      sensor_trigger_handler_t handler)
{
	struct bno055_data *data = dev->data;
	int err;
	LOG_INF("SET TRIGGER [%d][%d]", trig->type, trig->chan);

	if ((trig->type == SENSOR_TRIG_DATA_READY) && (trig->chan == SENSOR_CHAN_ACCEL_XYZ)) {
		err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_BSX_DRDY,
						  BNO055_IRQ_MASK_ACC_BSX_DRDY, handler != NULL);
		if (err < 0) {
			return err;
		}

		data->trigger_handler[BNO055_IRQ_ACC_BSX_DRDY] = handler;
		data->trigger[BNO055_IRQ_ACC_BSX_DRDY] = trig;
		return 0;
	}

	if ((trig->type == SENSOR_TRIG_DATA_READY) && (trig->chan == SENSOR_CHAN_MAGN_XYZ)) {
		err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_MAG_DRDY,
						  BNO055_IRQ_MASK_MAG_DRDY, handler != NULL);
		if (err < 0) {
			return err;
		}

		data->trigger_handler[BNO055_IRQ_MAG_DRDY] = handler;
		data->trigger[BNO055_IRQ_MAG_DRDY] = trig;
		return 0;
	}

	if ((trig->type == SENSOR_TRIG_DATA_READY) && (trig->chan == SENSOR_CHAN_GYRO_XYZ)) {
		err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_GYR_DRDY,
						  BNO055_IRQ_MASK_GYR_DRDY, handler != NULL);
		if (err < 0) {
			return err;
		}

		data->trigger_handler[BNO055_IRQ_GYR_DRDY] = handler;
		data->trigger[BNO055_IRQ_GYR_DRDY] = trig;
		return 0;
	}

	if ((trig->type == SENSOR_TRIG_DELTA) && BNO055_IS_GYRO_CHANNEL(trig->chan)) {
		if (trig->chan == SENSOR_CHAN_GYRO_XYZ) {
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_GYR_AM,
							  BNO055_IRQ_GYR_MASK_AN_MOTION_AXIS,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if (trig->chan == SENSOR_CHAN_GYRO_X) {
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_GYR_AM,
							  BNO055_IRQ_GYR_SETTINGS_AN_MOTION_X,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if (trig->chan == SENSOR_CHAN_GYRO_Y) {
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_GYR_AM,
							  BNO055_IRQ_GYR_SETTINGS_AN_MOTION_Y,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if (trig->chan == SENSOR_CHAN_GYRO_Z) {
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_GYR_AM,
							  BNO055_IRQ_GYR_SETTINGS_AN_MOTION_Z,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		}

		data->trigger_handler[BNO055_IRQ_GYR_AM] = handler;
		data->trigger[BNO055_IRQ_GYR_AM] = trig;
		return 0;
	}

	if ((trig->type == SENSOR_TRIG_DELTA) && BNO055_IS_ACCEL_CHANNEL(trig->chan)) {
		if ((data->trigger_handler[BNO055_IRQ_ACC_AN_MOTION] != NULL) &&
		    (data->trigger[BNO055_IRQ_ACC_AN_MOTION] != NULL)) {
			LOG_ERR("Any/No Motion trigger already affected!!");
			LOG_ERR("Clear the trigger with NULL callback to setup a new trigger!!");
			return -ENOTSUP;
		}

		if ((trig->chan == SENSOR_CHAN_ACCEL_XYZ)) {
			LOG_DBG("TRIGGER SET ACC_XYZ DELTA");
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_AM,
							  BNO055_IRQ_ACC_MASK_AN_MOTION_AXIS,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if ((trig->chan == SENSOR_CHAN_ACCEL_X)) {
			LOG_DBG("TRIGGER SET ACC_X DELTA");
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_AM,
							  BNO055_IRQ_ACC_SETTINGS_AN_MOTION_X,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if ((trig->chan == SENSOR_CHAN_ACCEL_Y)) {
			LOG_DBG("TRIGGER SET ACC_Y DELTA");
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_AM,
							  BNO055_IRQ_ACC_SETTINGS_AN_MOTION_Y,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if ((trig->chan == SENSOR_CHAN_ACCEL_Z)) {
			LOG_DBG("TRIGGER SET ACC_Z DELTA");
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_AM,
							  BNO055_IRQ_ACC_SETTINGS_AN_MOTION_Z,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		}

		data->trigger_handler[BNO055_IRQ_ACC_AN_MOTION] = handler;
		data->trigger[BNO055_IRQ_ACC_AN_MOTION] = trig;
		return 0;
	}

	if ((trig->type == SENSOR_TRIG_STATIONARY) && BNO055_IS_ACCEL_CHANNEL(trig->chan)) {
		if ((data->trigger_handler[BNO055_IRQ_ACC_AN_MOTION] != NULL) &&
		    (data->trigger[BNO055_IRQ_ACC_AN_MOTION] != NULL)) {
			LOG_ERR("Any/No Motion trigger already affected!!");
			LOG_ERR("Clear the trigger with NULL callback to setup a new trigger!!");
			return -ENOTSUP;
		}

		if ((trig->chan == SENSOR_CHAN_ACCEL_XYZ)) {
			LOG_DBG("TRIGGER SET ACC_XYZ DELTA");
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_NM,
							  BNO055_IRQ_ACC_MASK_AN_MOTION_AXIS,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if ((trig->chan == SENSOR_CHAN_ACCEL_X)) {
			LOG_DBG("TRIGGER SET ACC_X DELTA");
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_NM,
							  BNO055_IRQ_ACC_SETTINGS_AN_MOTION_X,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if ((trig->chan == SENSOR_CHAN_ACCEL_Y)) {
			LOG_DBG("TRIGGER SET ACC_Y DELTA");
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_NM,
							  BNO055_IRQ_ACC_SETTINGS_AN_MOTION_Y,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		} else if ((trig->chan == SENSOR_CHAN_ACCEL_Z)) {
			LOG_DBG("TRIGGER SET ACC_Z DELTA");
			err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_NM,
							  BNO055_IRQ_ACC_SETTINGS_AN_MOTION_Z,
							  handler != NULL);
			if (err < 0) {
				return err;
			}
		}

		data->trigger_handler[BNO055_IRQ_ACC_AN_MOTION] = handler;
		data->trigger[BNO055_IRQ_ACC_AN_MOTION] = trig;
		return 0;
	}

	if ((trig->type == (enum sensor_trigger_type)BNO055_SENSOR_TRIG_HIGH_G) &&
	    (trig->chan == SENSOR_CHAN_ACCEL_XYZ)) {
		err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_ACC_HIGH_G,
						  BNO055_IRQ_ACC_MASK_HG_AXIS, handler != NULL);
		if (err < 0) {
			return err;
		}

		data->trigger_handler[BNO055_IRQ_ACC_HIGH_G] = handler;
		data->trigger[BNO055_IRQ_ACC_HIGH_G] = trig;
		return 0;
	}

	if ((trig->type == (enum sensor_trigger_type)BNO055_SENSOR_TRIG_HIGH_RATE) &&
	    (trig->chan == SENSOR_CHAN_GYRO_XYZ)) {
		err = bno055_trigger_configuation(dev, trig, BNO055_IRQ_MASK_GYR_HIGH_RATE,
						  BNO055_IRQ_MASK_GYR_HIGH_RATE, handler != NULL);
		if (err < 0) {
			return err;
		}

		data->trigger_handler[BNO055_IRQ_GYR_HIGH_RATE] = handler;
		data->trigger[BNO055_IRQ_GYR_HIGH_RATE] = trig;
		return 0;
	}

	return -ENOTSUP;
}
#endif

static int bno055_init(const struct device *dev)
{
	const struct bno055_config *config = dev->config;
	struct bno055_data *data = dev->data;

	if (!i2c_is_ready_dt(&config->i2c_bus)) {
		LOG_ERR("I2C bus not ready!!");
		return -ENODEV;
	}

	LOG_INF("DEFERRED [%d]", config->deferred);
	if (!config->deferred) {
		k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
	}

	LOG_INF("CONFIG");
	LOG_INF("USE XTAL [%d]", config->use_xtal);
	int err;

	/* Switch to Page 0 */
	bno055_set_page(dev, BNO055_PAGE_ZERO);

	/* Send Reset Command */
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER,
				    BNO055_COMMAND_RESET);
	if (err < 0) {
		LOG_ERR("RESET write I2C Failed!!");
		return err;
	}
	data->mode = BNO055_MODE_CONFIG;
	k_sleep(K_MSEC(BNO055_TIMING_RESET_CONFIG));

	/* Check for chip id to validate the power on of the sensor */
	uint8_t reg;
	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_CHIP_ID, &reg);
	if (err < 0) {
		LOG_ERR("CHIP_ID read I2C Failed!!");
		return err;
	}
	LOG_INF("CHIP ID [%d]", reg);

	if (reg != BNO055_CHIP_ID) {
		LOG_WRN("BNO055 Not Ready yet!!");
		k_sleep(K_MSEC(BNO055_TIMING_RESET_CONFIG));

		err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_CHIP_ID, &reg);
		if (err < 0) {
			LOG_ERR("CHIP_ID read I2C Failed!!");
			return err;
		}
		if (reg != BNO055_CHIP_ID) {
			LOG_ERR("CHIP_ID Failed!!");
			return -ENODEV;
		}
	}

	uint8_t soft[2];
	err = i2c_burst_read_dt(&config->i2c_bus, BNO055_REGISTER_SOFTWARE_REV, soft, sizeof(soft));
	LOG_INF("SOFTWARE REV [%d][%d]", soft[1], soft[0]);

	/* Configure Unit according to Zephyr */
	uint8_t selection = (BNO055_ORIENTATION_WINDOWS << 7) | (BNO055_TEMP_UNIT_CELSIUS << 4) |
			    (BNO055_EULER_UNIT_RADIANS << 2) | (BNO055_GYRO_UNIT_RPS << 1) |
			    (BNO055_ACCEL_UNIT_MS_2 << 0);
	err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_UNIT_SELECT, selection);
	if (err < 0) {
		return err;
	}

	if (config->use_xtal) {
		err = i2c_reg_write_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER,
					    BNO055_COMMAND_XTAL);
		if (err < 0) {
			return err;
		}
	}

	err = i2c_reg_read_byte_dt(&config->i2c_bus, BNO055_REGISTER_SYS_TRIGGER, &reg);
	LOG_INF("SYS TRIGGER [%d]", reg);

	/* Configure GPIO interrupt */
#if BNO055_USE_IRQ
	if (!gpio_is_ready_dt(&config->irq_gpio)) {
		LOG_ERR("GPIO not ready!!");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
	if (err < 0) {
		LOG_ERR("Failed to configure GPIO!!");
		return err;
	}

	err = gpio_pin_interrupt_configure_dt(&config->irq_gpio, GPIO_INT_EDGE_RISING);
	if (err < 0) {
		LOG_ERR("Failed to configure interrupt!!");
		return err;
	}

	gpio_init_callback(&data->gpio_cb, bno055_gpio_callback_handler, BIT(config->irq_gpio.pin));

	err = gpio_add_callback_dt(&config->irq_gpio, &data->gpio_cb);
	if (err < 0) {
		LOG_ERR("Failed to add GPIO callback!!");
		return err;
	}
	LOG_INF("GPIO callback configured!!");

	data->dev = dev;
	memset(&(data->trigger_handler[0]), 0, sizeof(data->trigger_handler));
	memset(&(data->trigger[0]), 0, sizeof(data->trigger));
	data->cb_work.handler = bno055_work_cb;
#endif

	return 0;
}

static const struct sensor_driver_api bno055_driver_api = {
	.attr_set = bno055_attr_set,
	.sample_fetch = bno055_sample_fetch,
	.channel_get = bno055_channel_get,
#if BNO055_USE_IRQ
	.trigger_set = bno055_trigger_set,
#endif
};

#define BNO055_INIT(n)                                                                             \
	static struct bno055_config bno055_config_##n = {                                          \
		.i2c_bus = I2C_DT_SPEC_INST_GET(n),                                                \
		.use_xtal = DT_INST_PROP(n, use_xtal),                                             \
		IF_ENABLED(BNO055_USE_IRQ,                                                         \
			   (.irq_gpio = GPIO_DT_SPEC_INST_GET_OR(n, irq_gpios, {0}))),             \
		IF_ENABLED(DT_ANY_INST_HAS_PROP_STATUS_OKAY(zephyr_deferred_init),                 \
			   (.deferred = DT_INST_PROP(n, zephyr_deferred_init))),                   \
	};                                                                                         \
	static struct bno055_data bno055_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, bno055_init, NULL, &bno055_data_##n, &bno055_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &bno055_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BNO055_INIT)
