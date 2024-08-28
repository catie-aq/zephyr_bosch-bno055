/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

enum OperatingMode {
    CONFIG_MODE     = 0x00,
    ACC_ONLY        = 0x01,
    MAG_ONLY        = 0x02,
    GYRO_ONLY       = 0x03,
    ACC_MAG         = 0x04,
    ACC_GYRO        = 0x05,
    MAG_GYRO		= 0x06,
    ACC_MAG_GYRO    = 0x07,
    IMU             = 0x08, // Relative orientation from ACC + GYR | Fast calculation (high rate output)
    COMPASS         = 0x09,
    M4G             = 0x0A, // Like IMU but replace GYR by MAG | much less power consumption
    NDOF_FMC_OFF    = 0x0B,
    NDOF            = 0x0C  // Fast MAG calibration ON | slightly higher consumption than NDOF_FMC_OFF
};

static const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(bno0550));

int main(void)
{
	struct sensor_value value[3];

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 1;
	}

	struct sensor_value config = {
		.val1 = ACC_ONLY,
		.val2 = 0,
	};
	sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CONFIGURATION, &config);

	while (1) {
		sensor_sample_fetch(dev);

		// Example for ambiant temperature sensor
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, value);
		printk("X(m.s-2): %d.%06d Y(m.s-2): %d.%06d Z(m.s-2): %d.%06d\n",
				value[0].val1, value[0].val2,
				value[1].val1, value[1].val2,
				value[2].val1, value[2].val2);

		k_sleep(K_MSEC(100));
	}
}
