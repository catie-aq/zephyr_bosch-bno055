/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <bno055.h> // Required for custom SENSOR_CHAN_*

static const struct device *const dev = DEVICE_DT_GET(DT_NODELABEL(bno0550));

int main(void)
{
	struct sensor_value lia[3], grav[3], eul[3], quat[4], calib[4];

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 1;
	}

	struct sensor_value config = {
		.val1 = BNO055_MODE_NDOF,
		.val2 = 0,
	};
	sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CONFIGURATION, &config);
	config.val1 = BNO055_POWER_NORMAL;
	sensor_attr_set(dev, SENSOR_CHAN_ALL, BNO055_SENSOR_ATTR_POWER_MODE, &config);

	while (1) {
		sensor_sample_fetch(dev);

		// Example for Linear Acceleration and Gravity
		sensor_channel_get(dev, BNO055_SENSOR_CHAN_LINEAR_ACCEL_XYZ, lia);
		sensor_channel_get(dev, BNO055_SENSOR_CHAN_GRAVITY_XYZ, grav);
		sensor_channel_get(dev, BNO055_SENSOR_CHAN_EULER_YRP, eul);
		sensor_channel_get(dev, BNO055_SENSOR_CHAN_QUATERNION_WXYZ, quat);
		sensor_channel_get(dev, BNO055_SENSOR_CHAN_CALIBRATION_SGAM, calib);

		printk("LINACCEL: X(m.s-2)[%d.%06d] Y(m.s-2)[%d.%06d] Z(m.s-2)[%d.%06d]\n",
		       lia[0].val1, lia[0].val2, lia[1].val1, lia[1].val2, lia[2].val1,
		       lia[2].val2);
		printk("GRAVITY: X(m.s-2)[%d.%06d] Y(m.s-2)[%d.%06d] Z(m.s-2)[%d.%06d]\n",
		       grav[0].val1, grav[0].val2, grav[1].val1, grav[1].val2, grav[2].val1,
		       grav[2].val2);
		printk("EULER: X(rad.s-1)[%d.%06d] Y(rad.s-1)[%d.%06d] Z(rad.s-1)[%d.%06d]\n",
		       eul[0].val1, eul[0].val2, eul[1].val1, eul[1].val2, eul[2].val1,
		       eul[2].val2);
		printk("QUATERNION: W[%d.%06d] X[%d.%06d] Y[%d.%06d] Z[%d.%06d]\n", quat[0].val1,
		       quat[0].val2, quat[1].val1, quat[1].val2, quat[2].val1, quat[2].val2,
		       quat[2].val1, quat[2].val2);
		printk("CALIB: SYS[%d] GYR[%d] ACC[%d] MAG[%d]\n", calib[0].val1, calib[1].val1,
		       calib[2].val1, calib[3].val1);

		k_sleep(K_MSEC(500));
	}
}
