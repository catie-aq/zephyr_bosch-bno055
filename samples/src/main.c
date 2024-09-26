/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <bno055.h> // Required for custom SENSOR_CHAN_*

static const struct device *const bno055_dev = DEVICE_DT_GET(DT_NODELABEL(bno0550));
static struct sensor_trigger trig_acc_drdy;
static struct sensor_trigger trig_acc_asn;
static struct sensor_trigger trig_acc_hg;
static struct sensor_trigger trig_gyr_am;
static struct sensor_trigger trig_gyr_hr;

static bool trigger_an_motion = false;
static bool bno055_fusion = false;

void acc_drdy(const struct device *dev, const struct sensor_trigger *trigger)
{
	printk("BSX data ready!!\n");
}

void acc_asn(const struct device *dev, const struct sensor_trigger *trigger)
{
	if (trigger->type == SENSOR_TRIG_DELTA) {
		printk("ACC any motion interrupt!!\n");
	} else if (trigger->type == SENSOR_TRIG_STATIONARY) {
		printk("ACC no motion interrupt!!\n");
	} else {
		printk("Unknown interrupt!!\n");
	}
}

void acc_hg(const struct device *dev, const struct sensor_trigger *trigger)
{
	printk("ACC high G interrupt!!\n");
}

void gyr_am(const struct device *dev, const struct sensor_trigger *trigger)
{
	printk("GYR any motion interrupt!!\n");
}

void gyr_hr(const struct device *dev, const struct sensor_trigger *trigger)
{
	printk("GYR high RATE interrupt!!\n");
}

int main(void)
{
#if DEVICE_DT_DEFER(DT_NODELABEL(bno0550))
	k_sleep(K_MSEC(BNO055_TIMING_STARTUP));
	device_init(bno055_dev);
#endif

	if (!device_is_ready(bno055_dev)) {
		printk("Device %s is not ready\n", bno055_dev->name);
		return 1;
	}

	struct sensor_value config = {
		.val1 = (bno055_fusion) ? BNO055_MODE_NDOF : BNO055_MODE_ACC_MAG_GYRO,
		.val2 = 0,
	};
	sensor_attr_set(bno055_dev, SENSOR_CHAN_ALL, SENSOR_ATTR_CONFIGURATION, &config);
	config.val1 = BNO055_POWER_NORMAL;
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_ALL, BNO055_SENSOR_ATTR_POWER_MODE, &config);

	trig_acc_drdy.type = SENSOR_TRIG_DATA_READY;
	trig_acc_drdy.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(bno055_dev, &trig_acc_drdy, acc_drdy);

	if (trigger_an_motion) {
		config.val1 = BNO055_ACC_DURATION_AM;
		config.val2 = 0x02; // Value
		sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &config);
		config.val1 = BNO055_ACC_THRESHOLD_AM;
		config.val2 = 0x04; // Value
		sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &config);
		trig_acc_asn.type = SENSOR_TRIG_DELTA;
		trig_acc_asn.chan = SENSOR_CHAN_ACCEL_XYZ;
		sensor_trigger_set(bno055_dev, &trig_acc_asn, acc_asn);
	} else {
		config.val1 = BNO055_ACC_DURATION_NM;
		config.val2 = BNO055_IRQ_ACC_SN_MOTION_NO || BNO055_ACC_SN_DURATION_1_SECONDS;
		sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &config);
		config.val1 = BNO055_ACC_THRESHOLD_NM;
		config.val2 = 0x04; // Value
		sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &config);
		trig_acc_asn.type = SENSOR_TRIG_STATIONARY;
		trig_acc_asn.chan = SENSOR_CHAN_ACCEL_XYZ;
		sensor_trigger_set(bno055_dev, &trig_acc_asn, acc_asn);
	}

	config.val1 = BNO055_ACC_DURATION_HG;
	config.val2 = 0x7F; // Value (127 + 1) * 2 = 256 ms
	sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = BNO055_ACC_THRESHOLD_HG;
	config.val2 = 0x48; // Value
	sensor_attr_set(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SLOPE_TH, &config);
	trig_acc_hg.type = BNO055_SENSOR_TRIG_HIGH_G;
	trig_acc_hg.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(bno055_dev, &trig_acc_hg, acc_hg);

	config.val1 = 0x02; // Value
	config.val2 = BNO055_GYR_AM_AWAKE_DURATION_8_SAMPLES;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = 0x08; // Value
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SLOPE_TH, &config);
	trig_gyr_am.type = SENSOR_TRIG_DELTA;
	trig_gyr_am.chan = SENSOR_CHAN_GYRO_XYZ;
	sensor_trigger_set(bno055_dev, &trig_gyr_am, gyr_am);

	config.val1 = 0x0A; // Value
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_X, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = 0x0A; // Value
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_Y, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = 0x0A; // Value
	config.val2 = 0;
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_Z, SENSOR_ATTR_SLOPE_DUR, &config);
	config.val1 = 0x01; // Threshold
	config.val2 = 0x00; // Hysteresis
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_X, SENSOR_ATTR_HYSTERESIS, &config);
	config.val1 = 0x01; // Threshold
	config.val2 = 0x00; // Hysteresis
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_Y, SENSOR_ATTR_HYSTERESIS, &config);
	config.val1 = 0x01; // Threshold
	config.val2 = 0x00; // Hysteresis
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_Z, SENSOR_ATTR_HYSTERESIS, &config);
	trig_gyr_hr.type = BNO055_SENSOR_TRIG_HIGH_RATE;
	trig_gyr_hr.chan = SENSOR_CHAN_GYRO_XYZ;
	sensor_trigger_set(bno055_dev, &trig_gyr_hr, gyr_hr);

	config.val1 = BNO055_GYR_FILTER_OFF; // Deactive HR Filter
	config.val2 = BNO055_GYR_FILTER_OFF; // Deactive AM Filter
	sensor_attr_set(bno055_dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FEATURE_MASK, &config);

	while (1) {
		sensor_sample_fetch(bno055_dev);

		// Example for Linear Acceleration and Gravity
		if (!bno055_fusion) {
			struct sensor_value acc[3];
			sensor_channel_get(bno055_dev, SENSOR_CHAN_ACCEL_XYZ, acc);

			printk("ACCEL: X(m.s-2)[%d.%06d] Y(m.s-2)[%d.%06d] Z(m.s-2)[%d.%06d]\n",
			       acc[0].val1, acc[0].val2, acc[1].val1, acc[1].val2, acc[2].val1,
			       acc[2].val2);
		} else {
			struct sensor_value lia[3], grav[3], eul[3], quat[4], calib[4];
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_LINEAR_ACCEL_XYZ, lia);
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_GRAVITY_XYZ, grav);
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_EULER_YRP, eul);
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_QUATERNION_WXYZ, quat);
			sensor_channel_get(bno055_dev, BNO055_SENSOR_CHAN_CALIBRATION_SGAM, calib);

			printk("LINACCEL: X(m.s-2)[%d.%06d] Y(m.s-2)[%d.%06d] Z(m.s-2)[%d.%06d]\n",
			       lia[0].val1, lia[0].val2, lia[1].val1, lia[1].val2, lia[2].val1,
			       lia[2].val2);
			printk("GRAVITY: X(m.s-2)[%d.%06d] Y(m.s-2)[%d.%06d] Z(m.s-2)[%d.%06d]\n",
			       grav[0].val1, grav[0].val2, grav[1].val1, grav[1].val2, grav[2].val1,
			       grav[2].val2);
			printk("EULER: X(rad.s-1)[%d.%06d] Y(rad.s-1)[%d.%06d] "
			       "Z(rad.s-1)[%d.%06d]\n",
			       eul[0].val1, eul[0].val2, eul[1].val1, eul[1].val2, eul[2].val1,
			       eul[2].val2);
			printk("QUATERNION: W[%d.%06d] X[%d.%06d] Y[%d.%06d] Z[%d.%06d]\n",
			       quat[0].val1, quat[0].val2, quat[1].val1, quat[1].val2, quat[2].val1,
			       quat[2].val2, quat[2].val1, quat[2].val2);
			printk("CALIB: SYS[%d] GYR[%d] ACC[%d] MAG[%d]\n", calib[0].val1,
			       calib[1].val1, calib[2].val1, calib[3].val1);
		}

		k_sleep(K_MSEC(500));
	}
}
