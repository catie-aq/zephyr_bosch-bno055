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
	struct sensor_value value[3];

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 1;
	}

	struct sensor_value config = {
		.val1 = NDOF,
		.val2 = 0,
	};
	sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CONFIGURATION, &config);

	while (1) {
		sensor_sample_fetch(dev);

		// Example for ambiant temperature sensor
		sensor_channel_get(dev, SENSOR_CHAN_GRAVITY_XYZ, value);
		printk("X(m.s-2): %d.%06d Y(m.s-2): %d.%06d Z(m.s-2): %d.%06d\n",
				value[0].val1, value[0].val2,
				value[1].val1, value[1].val2,
				value[2].val1, value[2].val2);

		k_sleep(K_MSEC(100));
	}
}
