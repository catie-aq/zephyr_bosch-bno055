/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BNO055_H_
#define ZEPHYR_DRIVERS_SENSOR_BNO055_H_

/* BNO055 Specific channels */
enum bno055_sensor_channel {
	BNO055_SENSOR_CHAN_EULER_Y = SENSOR_CHAN_PRIV_START + 0,
	BNO055_SENSOR_CHAN_EULER_R = SENSOR_CHAN_PRIV_START + 1,
	BNO055_SENSOR_CHAN_EULER_P = SENSOR_CHAN_PRIV_START + 2,
	BNO055_SENSOR_CHAN_EULER_YRP = SENSOR_CHAN_PRIV_START + 3,

	BNO055_SENSOR_CHAN_QUATERNION_W = SENSOR_CHAN_PRIV_START + 4,
	BNO055_SENSOR_CHAN_QUATERNION_X = SENSOR_CHAN_PRIV_START + 5,
	BNO055_SENSOR_CHAN_QUATERNION_Y = SENSOR_CHAN_PRIV_START + 6,
	BNO055_SENSOR_CHAN_QUATERNION_Z = SENSOR_CHAN_PRIV_START + 7,
	BNO055_SENSOR_CHAN_QUATERNION_WXYZ = SENSOR_CHAN_PRIV_START + 8,

	BNO055_SENSOR_CHAN_LINEAR_ACCEL_X = SENSOR_CHAN_PRIV_START + 9,
	BNO055_SENSOR_CHAN_LINEAR_ACCEL_Y = SENSOR_CHAN_PRIV_START + 10,
	BNO055_SENSOR_CHAN_LINEAR_ACCEL_Z = SENSOR_CHAN_PRIV_START + 11,
	BNO055_SENSOR_CHAN_LINEAR_ACCEL_XYZ = SENSOR_CHAN_PRIV_START + 12,

	BNO055_SENSOR_CHAN_GRAVITY_X = SENSOR_CHAN_PRIV_START + 13,
	BNO055_SENSOR_CHAN_GRAVITY_Y = SENSOR_CHAN_PRIV_START + 14,
	BNO055_SENSOR_CHAN_GRAVITY_Z = SENSOR_CHAN_PRIV_START + 15,
	BNO055_SENSOR_CHAN_GRAVITY_XYZ = SENSOR_CHAN_PRIV_START + 16,

	BNO055_SENSOR_CHAN_CALIBRATION_SYS = SENSOR_CHAN_PRIV_START + 17,
	BNO055_SENSOR_CHAN_CALIBRATION_GYR = SENSOR_CHAN_PRIV_START + 18,
	BNO055_SENSOR_CHAN_CALIBRATION_ACC = SENSOR_CHAN_PRIV_START + 19,
	BNO055_SENSOR_CHAN_CALIBRATION_MAG = SENSOR_CHAN_PRIV_START + 20,
	BNO055_SENSOR_CHAN_CALIBRATION_SGAM = SENSOR_CHAN_PRIV_START + 21
};

/* BNO055 Specific attibute */
enum bno055_sensor_attribute {
	BNO055_SENSOR_ATTR_POWER_MODE = SENSOR_ATTR_PRIV_START + 0,
};

/* Registers (PAGE 0) */
#define BNO055_REGISTER_PAGE_ID 0x07

// Identification Registers
#define BNO055_REGISTER_CHIP_ID 0x00
#define BNO055_REGISTER_ACC_ID  0x01
#define BNO055_REGISTER_MAG_ID  0x02
#define BNO055_REGISTER_GYR_ID  0x03

// Revision Registers
#define BNO055_REGISTER_SOFTWARE_REV     BNO055_REGISTER_SOFTWARE_REV_LSB // 2 Octets
#define BNO055_REGISTER_SOFTWARE_REV_LSB 0x04
#define BNO055_REGISTER_SOFTWARE_REV_MSB 0x05
#define BNO055_REGISTER_BOOTLOADER_REV   0x06

// Accelerometer Registers
#define BNO055_REGISTER_ACC_DATA_X_LSB 0x08
#define BNO055_REGISTER_ACC_DATA_X_MSB 0x09
#define BNO055_REGISTER_ACC_DATA_Y_LSB 0x0A
#define BNO055_REGISTER_ACC_DATA_Y_MSB 0x0B
#define BNO055_REGISTER_ACC_DATA_Z_LSB 0x0C
#define BNO055_REGISTER_ACC_DATA_Z_MSB 0x0D
#define BNO055_REGISTER_ACC_DATA_X     BNO055_REGISTER_ACC_DATA_X_LSB
#define BNO055_REGISTER_ACC_DATA_Y     BNO055_REGISTER_ACC_DATA_Y_LSB
#define BNO055_REGISTER_ACC_DATA_Z     BNO055_REGISTER_ACC_DATA_Z_LSB
#define BNO055_REGISTER_ACC_DATA       BNO055_REGISTER_ACC_DATA_X

// Magnetometer Registers
#define BNO055_REGISTER_MAG_DATA_X_LSB 0x0E
#define BNO055_REGISTER_MAG_DATA_X_MSB 0x0F
#define BNO055_REGISTER_MAG_DATA_Y_LSB 0x10
#define BNO055_REGISTER_MAG_DATA_Y_MSB 0x11
#define BNO055_REGISTER_MAG_DATA_Z_LSB 0x12
#define BNO055_REGISTER_MAG_DATA_Z_MSB 0x13
#define BNO055_REGISTER_MAG_DATA_X     BNO055_REGISTER_MAG_DATA_X_LSB
#define BNO055_REGISTER_MAG_DATA_Y     BNO055_REGISTER_MAG_DATA_Y_LSB
#define BNO055_REGISTER_MAG_DATA_Z     BNO055_REGISTER_MAG_DATA_Z_LSB
#define BNO055_REGISTER_MAG_DATA       BNO055_REGISTER_MAG_DATA_X

// Gyroscope Registers
#define BNO055_REGISTER_GYR_DATA_X_LSB 0x14
#define BNO055_REGISTER_GYR_DATA_X_MSB 0x15
#define BNO055_REGISTER_GYR_DATA_Y_LSB 0x16
#define BNO055_REGISTER_GYR_DATA_Y_MSB 0x17
#define BNO055_REGISTER_GYR_DATA_Z_LSB 0x18
#define BNO055_REGISTER_GYR_DATA_Z_MSB 0x19
#define BNO055_REGISTER_GYR_DATA_X     BNO055_REGISTER_GYR_DATA_X_LSB
#define BNO055_REGISTER_GYR_DATA_Y     BNO055_REGISTER_GYR_DATA_Y_LSB
#define BNO055_REGISTER_GYR_DATA_Z     BNO055_REGISTER_GYR_DATA_Z_LSB
#define BNO055_REGISTER_GYR_DATA       BNO055_REGISTER_GYR_DATA_X

// Euler Registers
#define BNO055_REGISTER_EUL_DATA_HEADING_LSB 0x1A
#define BNO055_REGISTER_EUL_DATA_HEADING_MSB 0x1B
#define BNO055_REGISTER_EUL_DATA_ROLL_LSB    0x1C
#define BNO055_REGISTER_EUL_DATA_ROLL_MSB    0x1D
#define BNO055_REGISTER_EUL_DATA_PITCH_LSB   0x1E
#define BNO055_REGISTER_EUL_DATA_PITCH_MSB   0x1F
#define BNO055_REGISTER_EUL_DATA_HEADING     BNO055_REGISTER_EUL_DATA_HEADING_LSB
#define BNO055_REGISTER_EUL_DATA_ROLL        BNO055_REGISTER_EUL_DATA_ROLL_LSB
#define BNO055_REGISTER_EUL_DATA_PITCH       BNO055_REGISTER_EUL_DATA_PITCH_LSB
#define BNO055_REGISTER_EUL_DATA             BNO055_REGISTER_EUL_DATA_PITCH

// Quaternion Registers
#define BNO055_REGISTER_QUA_DATA_W_LSB 0x20
#define BNO055_REGISTER_QUA_DATA_W_MSB 0x21
#define BNO055_REGISTER_QUA_DATA_X_LSB 0x22
#define BNO055_REGISTER_QUA_DATA_X_MSB 0x23
#define BNO055_REGISTER_QUA_DATA_Y_LSB 0x24
#define BNO055_REGISTER_QUA_DATA_Y_MSB 0x25
#define BNO055_REGISTER_QUA_DATA_Z_LSB 0x26
#define BNO055_REGISTER_QUA_DATA_Z_MSB 0x27
#define BNO055_REGISTER_QUA_DATA_W     BNO055_REGISTER_QUA_DATA_W_LSB
#define BNO055_REGISTER_QUA_DATA_X     BNO055_REGISTER_QUA_DATA_X_LSB
#define BNO055_REGISTER_QUA_DATA_Y     BNO055_REGISTER_QUA_DATA_Y_LSB
#define BNO055_REGISTER_QUA_DATA_Z     BNO055_REGISTER_QUA_DATA_Z_LSB
#define BNO055_REGISTER_QUA_DATA       BNO055_REGISTER_QUA_DATA_W

// Linear Acceleration Registers
#define BNO055_REGISTER_LIA_DATA_X_LSB 0x28
#define BNO055_REGISTER_LIA_DATA_X_MSB 0x29
#define BNO055_REGISTER_LIA_DATA_Y_LSB 0x2A
#define BNO055_REGISTER_LIA_DATA_Y_MSB 0x2B
#define BNO055_REGISTER_LIA_DATA_Z_LSB 0x2C
#define BNO055_REGISTER_LIA_DATA_Z_MSB 0x2D
#define BNO055_REGISTER_LIA_DATA_X     BNO055_REGISTER_LIA_DATA_X_LSB
#define BNO055_REGISTER_LIA_DATA_Y     BNO055_REGISTER_LIA_DATA_Y_LSB
#define BNO055_REGISTER_LIA_DATA_Z     BNO055_REGISTER_LIA_DATA_Z_LSB
#define BNO055_REGISTER_LIA_DATA       BNO055_REGISTER_LIA_DATA_X

// Gravity Vector Registers
#define BNO055_REGISTER_GRV_DATA_X_LSB 0x2E
#define BNO055_REGISTER_GRV_DATA_X_MSB 0x2F
#define BNO055_REGISTER_GRV_DATA_Y_LSB 0x30
#define BNO055_REGISTER_GRV_DATA_Y_MSB 0x31
#define BNO055_REGISTER_GRV_DATA_Z_LSB 0x32
#define BNO055_REGISTER_GRV_DATA_Z_MSB 0x33
#define BNO055_REGISTER_GRV_DATA_X     BNO055_REGISTER_GRV_DATA_X_LSB
#define BNO055_REGISTER_GRV_DATA_Y     BNO055_REGISTER_GRV_DATA_Y_LSB
#define BNO055_REGISTER_GRV_DATA_Z     BNO055_REGISTER_GRV_DATA_Z_LSB
#define BNO055_REGISTER_GRV_DATA       BNO055_REGISTER_GRV_DATA_X

// Others
#define BNO055_REGISTER_TEMP               0x34
#define BNO055_REGISTER_CALIBRATION_STATUS 0x35 // [SYS(2)|GYR(2)|ACC(2)|MAG(2)]
#define BNO055_REGISTER_SELF_TEST_RESULT   0x36
#define BNO055_REGISTER_IRQ_STATUS         0x37
#define BNO055_REGISTER_SYS_CLK_STATUS     0x38
#define BNO055_REGISTER_SYS_STATUS         0x39
#define BNO055_REGISTER_SYS_ERROR          0x3A

#define BNO055_REGISTER_UNIT_SELECT    0x3B
#define BNO055_REGISTER_OPERATION_MODE 0x3D
#define BNO055_REGISTER_POWER_MODE     0x3E
#define BNO055_REGISTER_SYS_TRIGGER    0x3F
#define BNO055_REGISTER_TEMP_SOURCE    0x40
#define BNO055_REGISTER_AXIS_MAP_CONF  0x41
#define BNO055_REGISTER_AXIS_MAP_SIGN  0x42

// SIC Matrix Registers
#define BNO055_REGISTER_SIC_MATRIC_LSB0 0x43
#define BNO055_REGISTER_SIC_MATRIC_MSB0 0x44
#define BNO055_REGISTER_SIC_MATRIC_LSB1 0x45
#define BNO055_REGISTER_SIC_MATRIC_MSB1 0x46
#define BNO055_REGISTER_SIC_MATRIC_LSB2 0x47
#define BNO055_REGISTER_SIC_MATRIC_MSB2 0x48
#define BNO055_REGISTER_SIC_MATRIC_LSB3 0x49
#define BNO055_REGISTER_SIC_MATRIC_MSB3 0x4A
#define BNO055_REGISTER_SIC_MATRIC_LSB4 0x4B
#define BNO055_REGISTER_SIC_MATRIC_MSB4 0x4C
#define BNO055_REGISTER_SIC_MATRIC_LSB5 0x4D
#define BNO055_REGISTER_SIC_MATRIC_MSB5 0x4E
#define BNO055_REGISTER_SIC_MATRIC_LSB6 0x4F
#define BNO055_REGISTER_SIC_MATRIC_MSB6 0x50
#define BNO055_REGISTER_SIC_MATRIC_LSB7 0x51
#define BNO055_REGISTER_SIC_MATRIC_MSB7 0x52
#define BNO055_REGISTER_SIC_MATRIC_LSB8 0x53
#define BNO055_REGISTER_SIC_MATRIC_MSB8 0x54

// Accelerometer Offset Registers
#define BNO055_REGISTER_ACC_OFFSET_X_LSB 0x55
#define BNO055_REGISTER_ACC_OFFSET_X_MSB 0x56
#define BNO055_REGISTER_ACC_OFFSET_Y_LSB 0x57
#define BNO055_REGISTER_ACC_OFFSET_Y_MSB 0x58
#define BNO055_REGISTER_ACC_OFFSET_Z_LSB 0x59
#define BNO055_REGISTER_ACC_OFFSET_Z_MSB 0x5A

// Magnetometer Offset Registers
#define BNO055_REGISTER_MAG_OFFSET_X_LSB 0x5B
#define BNO055_REGISTER_MAG_OFFSET_X_MSB 0x5C
#define BNO055_REGISTER_MAG_OFFSET_Y_LSB 0x5D
#define BNO055_REGISTER_MAG_OFFSET_Y_MSB 0x5E
#define BNO055_REGISTER_MAG_OFFSET_Z_LSB 0x5F
#define BNO055_REGISTER_MAG_OFFSET_Z_MSB 0x60

// Gyroscope Offset Registers
#define BNO055_REGISTER_GYR_OFFSET_X_LSB 0x61
#define BNO055_REGISTER_GYR_OFFSET_X_MSB 0x62
#define BNO055_REGISTER_GYR_OFFSET_Y_LSB 0x63
#define BNO055_REGISTER_GYR_OFFSET_Y_MSB 0x64
#define BNO055_REGISTER_GYR_OFFSET_Z_LSB 0x65
#define BNO055_REGISTER_GYR_OFFSET_Z_MSB 0x66

#define BNO055_REGISTER_ACC_RADIUS_LSB 0x67
#define BNO055_REGISTER_ACC_RADIUS_MSB 0x68
#define BNO055_REGISTER_MAG_RADIUS_LSB 0x69
#define BNO055_REGISTER_MAG_RADIUS_MSB 0x6A

/* Registers (PAGE 1) */
#define BNO055_REGISTER_ACC_CONFIG       0x08
#define BNO055_REGISTER_MAG_CONFIG       0x09
#define BNO055_REGISTER_GYR_CONFIG_0     0x0A
#define BNO055_REGISTER_GYR_CONFIG_1     0x0B
#define BNO055_REGISTER_ACC_SLEEP_CONFIG 0x0C
#define BNO055_REGISTER_GYR_SLEEP_CONFIG 0x0D

#define BNO055_REGISTER_INT_MASK   0x0F
#define BNO055_REGISTER_INT_ENABLE 0x10

#define BNO055_REGISTER_ACC_ANY_MOTION_THRESHOLD   0x11
#define BNO055_REGISTER_ACC_INT_SETTINGS           0x12
#define BNO055_REGISTER_ACC_HIGH_GRAVITY_DURATION  0x13
#define BNO055_REGISTER_ACC_HIGH_GRAVITY_THRESHOLD 0x14
#define BNO055_REGISTER_ACC_NO_MOTION_THRESHOLD    0x15
#define BNO055_REGISTER_ACC_NO_MOTION_SET          0x16

#define BNO055_REGISTER_GYR_INT_SETTINGS    0x17
#define BNO055_REGISTER_GYR_HIGH_RATE_X_SET 0x18
#define BNO055_REGISTER_GYR_DURATION_X      0x19
#define BNO055_REGISTER_GYR_HIGH_RATE_Y_SET 0x1A
#define BNO055_REGISTER_GYR_DURATION_Y      0x1B
#define BNO055_REGISTER_GYR_HIGH_RATE_Z_SET 0x1C
#define BNO055_REGISTER_GYR_DURATION_Z      0x1D
#define BNO055_REGISTER_GYR_AM_THRESHOLD    0x1E
#define BNO055_REGISTER_GYR_AM_SET          0x1F

#define BNO055_CHIP_ID   0xA0
#define BNO055_RESET_INT (0x01 << 6)

/* Command Register */
#define BNO055_COMMAND_RESET 0x20
#define BNO055_COMMAND_XTAL  0x80

/* Timings */
#define BNO055_TIMING_STARTUP            400 // 400ms
#define BNO055_TIMING_RESET_CONFIG       650 // 650ms
#define BNO055_TIMING_SWITCH_FROM_CONFIG 10  // 7 ms
#define BNO055_TIMING_SWITCH_FROM_ANY    20  // 19 ms

/* Unit Selection Configuration */
#define BNO055_ORIENTATION_WINDOWS 0x00 // clockwise pitch
#define BNO055_TEMP_UNIT_CELSIUS   0x00 // CELSIUS
#define BNO055_EULER_UNIT_RADIANS  0x01 // RADIANS
#define BNO055_GYRO_UNIT_RPS       0x01 // RPS
#define BNO055_ACCEL_UNIT_MS_2     0x00 // MS_2

/* Unit Resolution */
#define BNO055_EULER_RESOLUTION      900   // RADIANS
#define BNO055_QUATERNION_RESOLUTION 16384 // Unitless
#define BNO055_UTESLA_TO_GAUSS       100
#define BNO055_UTESLA_RESOLUTION     16
#define BNO055_GYRO_RESOLUTION       900 // RPS
#define BNO055_ACCEL_RESOLUTION      100 // MS_2

/* Accelerometer Configuration */
#define BNO055_ACC_2G  0x00
#define BNO055_ACC_4G  0x01
#define BNO055_ACC_8G  0x02
#define BNO055_ACC_16G 0x03

#define BNO055_ACC_8Hz    0x00
#define BNO055_ACC_16Hz   0x01
#define BNO055_ACC_31Hz   0x02
#define BNO055_ACC_62Hz   0x03
#define BNO055_ACC_125Hz  0x04
#define BNO055_ACC_250Hz  0x05
#define BNO055_ACC_500Hz  0x06
#define BNO055_ACC_1000Hz 0x07

#define BNO055_ACC_NORMAL       0x00
#define BNO055_ACC_SUSPEND      0x01
#define BNO055_ACC_LOW_POWER_1  0x02
#define BNO055_ACC_STANDBY      0x03
#define BNO055_ACC_LOW_POWER_2  0x04
#define BNO055_ACC_DEEP_SUSPEND 0x05

/* Magnetometer Configuration */
#define BNO055_MAG_2Hz  0x00
#define BNO055_MAG_6Hz  0x01
#define BNO055_MAG_8Hz  0x02
#define BNO055_MAG_10Hz 0x03
#define BNO055_MAG_15Hz 0x04
#define BNO055_MAG_20Hz 0x05
#define BNO055_MAG_25Hz 0x06
#define BNO055_MAG_30Hz 0x07

#define BNO055_MAG_LOW_POWER        0x00
#define BNO055_MAG_REGULAR          0x01
#define BNO055_MAG_ENHANCED_REGULAR 0x02
#define BNO055_MAG_HIGH_ACCURACY    0x03

#define BNO055_MAG_NORMAL     0x00
#define BNO055_MAG_SLEEP      0x01
#define BNO055_MAG_SUSPEND    0x02
#define BNO055_MAG_FORCE_MODE 0x03

/* Gyroscope Configuration */
#define BNO055_GYR_2000DPS 0x00
#define BNO055_GYR_1000DPS 0x01
#define BNO055_GYR_500DPS  0x02
#define BNO055_GYR_250DPS  0x03
#define BNO055_GYR_125DPS  0x04

#define BNO055_GYR_523Hz 0x00
#define BNO055_GYR_230Hz 0x01
#define BNO055_GYR_116Hz 0x02
#define BNO055_GYR_47Hz  0x03
#define BNO055_GYR_23Hz  0x04
#define BNO055_GYR_12Hz  0x05
#define BNO055_GYR_64Hz  0x06
#define BNO055_GYR_32Hz  0x07

#define BNO055_GYR_NORMAL             0x00
#define BNO055_GYR_FAST_POWER_UP      0x01
#define BNO055_GYR_DEEP_SUSPEND       0x02
#define BNO055_GYR_SUSPEND            0x03
#define BNO055_GYR_ADVANCED_POWERSAVE 0x04

/* Custom Sensors Configuration */
#if defined(CONFIG_BNO055_ACC_CUSTOM_CONFIG)
#if defined(CONFIG_BNO055_ACC_2G_RANGE)
#define BNO055_ACC_RANGE BNO055_ACC_2G
#elif defined(CONFIG_BNO055_ACC_4G_RANGE)
#define BNO055_ACC_RANGE BNO055_ACC_4G
#elif defined(CONFIG_BNO055_ACC_8G_RANGE)
#define BNO055_ACC_RANGE BNO055_ACC_8G
#elif defined(CONFIG_BNO055_ACC_16G_RANGE)
#define BNO055_ACC_RANGE BNO055_ACC_16G
#endif

#if defined(CONFIG_BNO055_ACC_8HZ_BANDWIDTH)
#define BNO055_ACC_BANDWIDTH BNO055_ACC_8Hz
#elif defined(CONFIG_BNO055_ACC_16HZ_BANDWIDTH)
#define BNO055_ACC_BANDWIDTH BNO055_ACC_16Hz
#elif defined(CONFIG_BNO055_ACC_31HZ_BANDWIDTH)
#define BNO055_ACC_BANDWIDTH BNO055_ACC_31Hz
#elif defined(CONFIG_BNO055_ACC_62HZ_BANDWIDTH)
#define BNO055_ACC_BANDWIDTH BNO055_ACC_62Hz
#elif defined(CONFIG_BNO055_ACC_125HZ_BANDWIDTH)
#define BNO055_ACC_BANDWIDTH BNO055_ACC_125Hz
#elif defined(CONFIG_BNO055_ACC_250HZ_BANDWIDTH)
#define BNO055_ACC_BANDWIDTH BNO055_ACC_250Hz
#elif defined(CONFIG_BNO055_ACC_500HZ_BANDWIDTH)
#define BNO055_ACC_BANDWIDTH BNO055_ACC_500Hz
#elif defined(CONFIG_BNO055_ACC_1000HZ_BANDWIDTH)
#define BNO055_ACC_BANDWIDTH BNO055_ACC_1000Hz
#endif

#if defined(CONFIG_BNO055_ACC_NORMAL_POWER)
#define BNO055_ACC_POWER BNO055_ACC_NORMAL
#elif defined(CONFIG_BNO055_ACC_LOW_1_POWER)
#define BNO055_ACC_POWER BNO055_ACC_LOW_POWER_1
#elif defined(CONFIG_BNO055_ACC_LOW_2_POWER)
#define BNO055_ACC_POWER BNO055_ACC_LOW_POWER_2
#elif defined(CONFIG_BNO055_ACC_STANDBY_POWER)
#define BNO055_ACC_POWER BNO055_ACC_STANDBY
#elif defined(CONFIG_BNO055_ACC_SUSPEND_POWER)
#define BNO055_ACC_POWER BNO055_ACC_SUSPEND
#elif defined(CONFIG_BNO055_ACC_DEEP_SUSPEND_POWER)
#define BNO055_ACC_POWER BNO055_ACC_DEEP_SUSPEND
#endif

#else
#define BNO055_ACC_RANGE     BNO055_ACC_4G
#define BNO055_ACC_BANDWIDTH BNO055_ACC_62Hz
#define BNO055_ACC_POWER     BNO055_ACC_NORMAL
#endif

#if defined(CONFIG_BNO055_MAG_CUSTOM_CONFIG)
#if defined(CONFIG_BNO055_MAG_2HZ_RATE)
#define BNO055_MAG_RATE BNO055_MAG_2Hz
#elif defined(CONFIG_BNO055_MAG_6HZ_RATE)
#define BNO055_MAG_RATE BNO055_MAG_6Hz
#elif defined(CONFIG_BNO055_MAG_8HZ_RATE)
#define BNO055_MAG_RATE BNO055_MAG_8Hz
#elif defined(CONFIG_BNO055_MAG_10HZ_RATE)
#define BNO055_MAG_RATE BNO055_MAG_10Hz
#elif defined(CONFIG_BNO055_MAG_15HZ_RATE)
#define BNO055_MAG_RATE BNO055_MAG_15Hz
#elif defined(CONFIG_BNO055_MAG_20HZ_RATE)
#define BNO055_MAG_RATE BNO055_MAG_20Hz
#elif defined(CONFIG_BNO055_MAG_25HZ_RATE)
#define BNO055_MAG_RATE BNO055_MAG_25Hz
#elif defined(CONFIG_BNO055_MAG_30HZ_RATE)
#define BNO055_MAG_RATE BNO055_MAG_30Hz
#endif

#if defined(CONFIG_BNO055_MAG_LOW_POWER_MODE)
#define BNO055_MAG_MODE BNO055_MAG_LOW_POWER
#elif defined(CONFIG_BNO055_MAG_REGULAR_MODE)
#define BNO055_MAG_MODE BNO055_MAG_REGULAR
#elif defined(CONFIG_BNO055_MAG_ENHANCED_REGULAR_MODE)
#define BNO055_MAG_MODE BNO055_MAG_ENHANCED_REGULAR
#elif defined(CONFIG_BNO055_MAG_HIGH_ACCURACY_MODE)
#define BNO055_MAG_MODE BNO055_MAG_HIGH_ACCURACY
#endif

#if defined(CONFIG_BNO055_MAG_NORMAL_POWER)
#define BNO055_MAG_POWER BNO055_MAG_NORMAL
#elif defined(CONFIG_BNO055_MAG_SUSPEND_POWER)
#define BNO055_MAG_POWER BNO055_MAG_SUSPEND
#elif defined(CONFIG_BNO055_MAG_SLEEP_POWER)
#define BNO055_MAG_POWER BNO055_MAG_SLEEP
#elif defined(CONFIG_BNO055_MAG_FORCED_POWER)
#define BNO055_MAG_POWER BNO055_MAG_FORCE_MODE
#endif

#else
#define BNO055_MAG_RATE  BNO055_MAG_20Hz
#define BNO055_MAG_MODE  BNO055_MAG_REGULAR
#define BNO055_MAG_POWER BNO055_MAG_FORCE_MODE
#endif

#if defined(CONFIG_BNO055_GYR_CUSTOM_CONFIG)
#if defined(CONFIG_BNO055_GYR_125_RANGE)
#define BNO055_GYR_RANGE BNO055_GYR_125DPS
#elif defined(CONFIG_BNO055_GYR_250_RANGE)
#define BNO055_GYR_RANGE BNO055_GYR_250DPS
#elif defined(CONFIG_BNO055_GYR_500_RANGE)
#define BNO055_GYR_RANGE BNO055_GYR_500DPS
#elif defined(CONFIG_BNO055_GYR_1000_RANGE)
#define BNO055_GYR_RANGE BNO055_GYR_1000DPS
#elif defined(CONFIG_BNO055_GYR_2000_RANGE)
#define BNO055_GYR_RANGE BNO055_GYR_2000DPS
#endif

#if defined(CONFIG_BNO055_GYR_12HZ_BANDWIDTH)
#define BNO055_GYR_BANDWIDTH BNO055_GYR_12Hz
#elif defined(CONFIG_BNO055_GYR_23HZ_BANDWIDTH)
#define BNO055_GYR_BANDWIDTH BNO055_GYR_23Hz
#elif defined(CONFIG_BNO055_GYR_32HZ_BANDWIDTH)
#define BNO055_GYR_BANDWIDTH BNO055_GYR_32Hz
#elif defined(CONFIG_BNO055_GYR_47HZ_BANDWIDTH)
#define BNO055_GYR_BANDWIDTH BNO055_GYR_47Hz
#elif defined(CONFIG_BNO055_GYR_64HZ_BANDWIDTH)
#define BNO055_GYR_BANDWIDTH BNO055_GYR_64Hz
#elif defined(CONFIG_BNO055_GYR_116HZ_BANDWIDTH)
#define BNO055_GYR_BANDWIDTH BNO055_GYR_116Hz
#elif defined(CONFIG_BNO055_GYR_230HZ_BANDWIDTH)
#define BNO055_GYR_BANDWIDTH BNO055_GYR_230Hz
#elif defined(CONFIG_BNO055_GYR_523HZ_BANDWIDTH)
#define BNO055_GYR_BANDWIDTH BNO055_GYR_523Hz
#endif

#if defined(CONFIG_BNO055_GYR_NORMAL_POWER)
#define BNO055_GYR_POWER BNO055_GYR_NORMAL
#elif defined(CONFIG_BNO055_GYR_FAST_POWER)
#define BNO055_GYR_POWER BNO055_GYR_FAST_POWER_UP
#elif defined(CONFIG_BNO055_GYR_SUSPEND_POWER)
#define BNO055_GYR_POWER BNO055_GYR_SUSPEND
#elif defined(CONFIG_BNO055_GYR_DEEP_SUSPEND_POWER)
#define BNO055_GYR_POWER BNO055_GYR_DEEP_SUSPEND
#elif defined(CONFIG_BNO055_GYR_POWERSAVE_POWER)
#define BNO055_GYR_POWER BNO055_GYR_ADVANCED_POWERSAVE
#endif

#else
#define BNO055_GYR_RANGE     BNO055_GYR_2000DPS
#define BNO055_GYR_BANDWIDTH BNO055_GYR_32Hz
#define BNO055_GYR_POWER     BNO055_GYR_NORMAL
#endif

/* BNO055 Configuration */
#define BNO055_PAGE_ID_MASK 0xFF
enum PageId {
	BNO055_PAGE_ZERO = 0x00,
	BNO055_PAGE_ONE = 0x01
};

#define BNO055_POWER_MODE_MASK 0x03
enum PowerMode {
	BNO055_POWER_NORMAL = 0x00,
	BNO055_POWER_LOW_POWER = 0x01,
	BNO055_POWER_SUSPEND = 0x02,
	BNO055_POWER_INVALID = 0x03
};

#define BNO055_OPERATION_MODE_MASK 0x0F
enum OperatingMode {
	BNO055_MODE_CONFIG = 0x00,
	BNO055_MODE_ACC_ONLY = 0x01,
	BNO055_MODE_MAG_ONLY = 0x02,
	BNO055_MODE_GYRO_ONLY = 0x03,
	BNO055_MODE_ACC_MAG = 0x04,
	BNO055_MODE_ACC_GYRO = 0x05,
	BNO055_MODE_MAG_GYRO = 0x06,
	BNO055_MODE_ACC_MAG_GYRO = 0x07,
	BNO055_MODE_IMU =
		0x08, // Relative orientation from ACC + GYR | Fast calculation (high rate output)
	BNO055_MODE_COMPASS = 0x09,
	BNO055_MODE_M4G = 0x0A, // Like IMU but replace GYR by MAG | much less power consumption
	BNO055_MODE_NDOF_FMC_OFF = 0x0B,
	BNO055_MODE_NDOF =
		0x0C // Fast MAG calibration ON | slightly higher consumption than NDOF_FMC_OFF
};

/* Data structures */
struct offset_data {
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
	int16_t gyr_x;
	int16_t gyr_y;
	int16_t gyr_z;
	int16_t acc_radius;
	int16_t mag_radius;
};

struct vector3_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

struct vector4_data {
	int16_t w;
	int16_t x;
	int16_t y;
	int16_t z;
};

struct calib_data {
	int16_t sys;
	int16_t gyr;
	int16_t acc;
	int16_t mag;
};

#define BNO055_CALIBRATION_FULL 0x03
#define BNO055_CALIBRATION_NONE 0x00

/* BNO055 Triggers */
#define BNO055_IRQ_ACC_BSX_DRDY  0
#define BNO055_IRQ_MAG_DRDY      1
#define BNO055_IRQ_GYR_AM        2
#define BNO055_IRQ_GYR_HIGH_RATE 3
#define BNO055_IRQ_GYR_DRDY      4
#define BNO055_IRQ_ACC_HIGH_G    5
#define BNO055_IRQ_ACC_AM        6
#define BNO055_IRQ_ACC_NM        7
#define BNO055_IRQ_ACC_AN_MOTION BNO055_IRQ_ACC_AM

#define BNO055_IRQ_MASK_ACC_BSX_DRDY  (0x01 << BNO055_IRQ_ACC_BSX_DRDY)
#define BNO055_IRQ_MASK_MAG_DRDY      (0x01 << BNO055_IRQ_MAG_DRDY)
#define BNO055_IRQ_MASK_GYR_AM        (0x01 << BNO055_IRQ_GYR_AM)
#define BNO055_IRQ_MASK_GYR_HIGH_RATE (0x01 << BNO055_IRQ_GYR_HIGH_RATE)
#define BNO055_IRQ_MASK_GYR_DRDY      (0x01 << BNO055_IRQ_GYR_DRDY)
#define BNO055_IRQ_MASK_ACC_HIGH_G    (0x01 << BNO055_IRQ_ACC_HIGH_G)
#define BNO055_IRQ_MASK_ACC_AM        (0x01 << BNO055_IRQ_ACC_AM)
#define BNO055_IRQ_MASK_ACC_NM        (0x01 << BNO055_IRQ_ACC_NM)

#define BNO055_IRQ_SIZE    7
#define BNO055_IRQ_DISABLE 0x00
#define BNO055_IRQ_ENABLE  0xFF

/* ACC Settings FLAGS */
#define BNO055_IRQ_ACC_NO_SHIFT  0
#define BNO055_IRQ_ACC_SHIFT_AM  BNO055_IRQ_ACC_NO_SHIFT
#define BNO055_IRQ_ACC_SHIFT_SNM 1

#define BNO055_IRQ_ACC_SETTINGS_AM_DUR      (0x03 << BNO055_IRQ_ACC_SHIFT_AM)
#define BNO055_IRQ_ACC_SETTINGS_AN_MOTION_X (0x01 << 2)
#define BNO055_IRQ_ACC_SETTINGS_AN_MOTION_Y (0x01 << 3)
#define BNO055_IRQ_ACC_SETTINGS_AN_MOTION_Z (0x01 << 4)
#define BNO055_IRQ_ACC_SETTINGS_HG_X        (0x01 << 5)
#define BNO055_IRQ_ACC_SETTINGS_HG_Y        (0x01 << 6)
#define BNO055_IRQ_ACC_SETTINGS_HG_Z        (0x01 << 7)

#define BNO055_IRQ_ACC_MASK_THRESHOLD   0xFF
#define BNO055_IRQ_ACC_MASK_AM_DURATION BNO055_IRQ_ACC_SETTINGS_AM_DUR
#define BNO055_IRQ_ACC_MASK_AN_MOTION_AXIS                                                         \
	(BNO055_IRQ_ACC_SETTINGS_AN_MOTION_X | BNO055_IRQ_ACC_SETTINGS_AN_MOTION_Y |               \
	 BNO055_IRQ_ACC_SETTINGS_AN_MOTION_Z)

#define BNO055_IRQ_ACC_MASK_SNM_THRESHOLD 0xFF
#define BNO055_IRQ_ACC_MASK_SNM_DURATION  (0x3F << BNO055_IRQ_ACC_SHIFT_SNM)
#define BNO055_IRQ_ACC_MASK_SLOW_DURATION (0x03 << BNO055_IRQ_ACC_SHIFT_SNM)
#define BNO055_IRQ_ACC_MASK_SNM_SET       0x01

#define BNO055_IRQ_ACC_MASK_HG_DURATION  0xFF
#define BNO055_IRQ_ACC_MASK_HG_THRESHOLD 0xFF
#define BNO055_IRQ_ACC_MASK_HG_AXIS                                                                \
	(BNO055_IRQ_ACC_SETTINGS_HG_X | BNO055_IRQ_ACC_SETTINGS_HG_Y | BNO055_IRQ_ACC_SETTINGS_HG_Z)

enum acc_threshold_type {
	BNO055_ACC_THRESHOLD_AM = 0x00,
	BNO055_ACC_THRESHOLD_NM = 0x01,
	BNO055_ACC_THRESHOLD_HG = 0x02,
};

enum acc_duration_type {
	BNO055_ACC_DURATION_AM = 0x00,
	BNO055_ACC_DURATION_NM = 0x01,
	BNO055_ACC_DURATION_HG = 0x02,
};

#define BNO055_IRQ_ACC_SNM_SHIFT      31
#define BNO055_IRQ_ACC_SN_MOTION_NO   (0x01 << BNO055_IRQ_ACC_SNM_SHIFT)
#define BNO055_IRQ_ACC_SN_MOTION_SLOW 0x00

enum acc_sn_motion_duration {
	BNO055_ACC_SN_DURATION_1_SECONDS = 0x01,
	BNO055_ACC_SN_DURATION_2_SECONDS = 0x02,
	BNO055_ACC_SN_DURATION_3_SECONDS = 0x03,
	BNO055_ACC_SN_DURATION_4_SECONDS = 0x04,
	BNO055_ACC_SN_DURATION_5_SECONDS = 0x05,
	BNO055_ACC_SN_DURATION_6_SECONDS = 0x06,
	BNO055_ACC_SN_DURATION_7_SECONDS = 0x07,
	BNO055_ACC_SN_DURATION_8_SECONDS = 0x08,
	BNO055_ACC_SN_DURATION_9_SECONDS = 0x09,
	BNO055_ACC_SN_DURATION_10_SECONDS = 0x0A,
	BNO055_ACC_SN_DURATION_11_SECONDS = 0x0B,
	BNO055_ACC_SN_DURATION_12_SECONDS = 0x0C,
	BNO055_ACC_SN_DURATION_13_SECONDS = 0x0D,
	BNO055_ACC_SN_DURATION_14_SECONDS = 0x0E,
	BNO055_ACC_SN_DURATION_15_SECONDS = 0x0F,
	BNO055_ACC_SN_DURATION_16_SECONDS = 0x10,

	BNO055_ACC_SN_DURATION_20_SECONDS = 0x14,
	BNO055_ACC_SN_DURATION_24_SECONDS = 0x18,
	BNO055_ACC_SN_DURATION_28_SECONDS = 0x1C,
	BNO055_ACC_SN_DURATION_32_SECONDS = 0x20,
	BNO055_ACC_SN_DURATION_36_SECONDS = 0x24,
	BNO055_ACC_SN_DURATION_40_SECONDS = 0x28,
	BNO055_ACC_SN_DURATION_44_SECONDS = 0x2C,
	BNO055_ACC_SN_DURATION_48_SECONDS = 0x30,
	BNO055_ACC_SN_DURATION_52_SECONDS = 0x34,
	BNO055_ACC_SN_DURATION_56_SECONDS = 0x38,
	BNO055_ACC_SN_DURATION_60_SECONDS = 0x3C,
	BNO055_ACC_SN_DURATION_64_SECONDS = 0x40,
	BNO055_ACC_SN_DURATION_68_SECONDS = 0x44,
	BNO055_ACC_SN_DURATION_72_SECONDS = 0x48,
	BNO055_ACC_SN_DURATION_76_SECONDS = 0x4C,
	BNO055_ACC_SN_DURATION_80_SECONDS = 0x50,

	BNO055_ACC_SN_DURATION_88_SECONDS = 0x58,
	BNO055_ACC_SN_DURATION_96_SECONDS = 0x60,
	BNO055_ACC_SN_DURATION_104_SECONDS = 0x68,
	BNO055_ACC_SN_DURATION_112_SECONDS = 0x70,
	BNO055_ACC_SN_DURATION_120_SECONDS = 0x78,
	BNO055_ACC_SN_DURATION_128_SECONDS = 0x80,
	BNO055_ACC_SN_DURATION_136_SECONDS = 0x88,
	BNO055_ACC_SN_DURATION_144_SECONDS = 0x90,
	BNO055_ACC_SN_DURATION_152_SECONDS = 0x98,
	BNO055_ACC_SN_DURATION_160_SECONDS = 0xA0,
	BNO055_ACC_SN_DURATION_168_SECONDS = 0xA8,
	BNO055_ACC_SN_DURATION_176_SECONDS = 0xB0,
	BNO055_ACC_SN_DURATION_184_SECONDS = 0xB8,
	BNO055_ACC_SN_DURATION_192_SECONDS = 0xC0,
	BNO055_ACC_SN_DURATION_200_SECONDS = 0xC8,
	BNO055_ACC_SN_DURATION_208_SECONDS = 0xD0,
	BNO055_ACC_SN_DURATION_216_SECONDS = 0xD8,
	BNO055_ACC_SN_DURATION_224_SECONDS = 0xE0,
	BNO055_ACC_SN_DURATION_232_SECONDS = 0xE8,
	BNO055_ACC_SN_DURATION_240_SECONDS = 0xF0,
	BNO055_ACC_SN_DURATION_248_SECONDS = 0xF8,
	BNO055_ACC_SN_DURATION_256_SECONDS = 0x100,
	BNO055_ACC_SN_DURATION_264_SECONDS = 0x108,
	BNO055_ACC_SN_DURATION_272_SECONDS = 0x110,
	BNO055_ACC_SN_DURATION_280_SECONDS = 0x118,
	BNO055_ACC_SN_DURATION_288_SECONDS = 0x120,
	BNO055_ACC_SN_DURATION_296_SECONDS = 0x128,
	BNO055_ACC_SN_DURATION_304_SECONDS = 0x130,
	BNO055_ACC_SN_DURATION_312_SECONDS = 0x138,
	BNO055_ACC_SN_DURATION_320_SECONDS = 0x140,
	BNO055_ACC_SN_DURATION_328_SECONDS = 0x148,
	BNO055_ACC_SN_DURATION_336_SECONDS = 0x150,
};

#define BNO055_IS_ACCEL_CHANNEL(val)                                                               \
	((val == SENSOR_CHAN_ACCEL_XYZ) || (val == SENSOR_CHAN_ACCEL_X) ||                         \
	 (val == SENSOR_CHAN_ACCEL_Y) || (val == SENSOR_CHAN_ACCEL_Z))

/* GYR Settings FLAGS */
#define BNO055_IRQ_GYR_NO_SHIFT                0
#define BNO055_IRQ_GYR_SHIFT_HR_FILT           7
#define BNO055_IRQ_GYR_SHIFT_AM_FILT           6
#define BNO055_IRQ_GYR_SHIFT_AM_THRESHOLD      0
#define BNO055_IRQ_GYR_SHIFT_AM_AWAKE_DURATION 2
#define BNO055_IRQ_GYR_SHIFT_AM_SAMPLES        0
#define BNO055_IRQ_GYR_SHIFT_HR_HYSTERESIS     5

#define BNO055_IRQ_GYR_SETTINGS_AN_MOTION_X (0x01 << 0)
#define BNO055_IRQ_GYR_SETTINGS_AN_MOTION_Y (0x01 << 1)
#define BNO055_IRQ_GYR_SETTINGS_AN_MOTION_Z (0x01 << 2)
#define BNO055_IRQ_GYR_SETTINGS_HR_X        (0x01 << 3)
#define BNO055_IRQ_GYR_SETTINGS_HR_Y        (0x01 << 4)
#define BNO055_IRQ_GYR_SETTINGS_HR_Z        (0x01 << 5)
#define BNO055_IRQ_GYR_SETTINGS_AM_FILT     (0x01 << BNO055_IRQ_GYR_SHIFT_AM_FILT)
#define BNO055_IRQ_GYR_SETTINGS_HR_FILT     (0x01 << BNO055_IRQ_GYR_SHIFT_HR_FILT)

#define BNO055_IRQ_GYR_MASK_AM_THRESHOLD      0x7F
#define BNO055_IRQ_GYR_MASK_AM_AWAKE_DURATION 0x7F
#define BNO055_IRQ_GYR_MASK_AM_SAMPLES        0x7F
#define BNO055_IRQ_GYR_MASK_AN_MOTION_AXIS                                                         \
	(BNO055_IRQ_GYR_SETTINGS_AN_MOTION_X | BNO055_IRQ_GYR_SETTINGS_AN_MOTION_Y |               \
	 BNO055_IRQ_GYR_SETTINGS_AN_MOTION_Z)

enum gyr_filter {
	BNO055_GYR_FILTER_ON = 0x00,
	BNO055_GYR_FILTER_OFF = 0x01,
};

enum gyr_am_awake {
	BNO055_GYR_AM_AWAKE_DURATION_8_SAMPLES = 0x00,
	BNO055_GYR_AM_AWAKE_DURATION_16_SAMPLES = 0x01,
	BNO055_GYR_AM_AWAKE_DURATION_32_SAMPLES = 0x02,
	BNO055_GYR_AM_AWAKE_DURATION_64_SAMPLES = 0x03,
};

#define BNO055_IRQ_GYR_MASK_HR_THRESHOLD  (0x1F << BNO055_IRQ_ACC_NO_SHIFT)
#define BNO055_IRQ_GYR_MASK_HR_HYSTERESIS (0x03 << BNO055_IRQ_GYR_SHIFT_HR_HYSTERESIS)
#define BNO055_IRQ_GYR_MASK_HR_DURATION   0xFF
#define BNO055_IRQ_GYR_MASK_HR_AXIS                                                                \
	(BNO055_IRQ_GYR_SETTINGS_HR_X | BNO055_IRQ_GYR_SETTINGS_HR_Y | BNO055_IRQ_GYR_SETTINGS_HR_Z)

#define BNO055_IS_GYRO_CHANNEL(val)                                                                \
	((val == SENSOR_CHAN_GYRO_XYZ) || (val == SENSOR_CHAN_GYRO_X) ||                           \
	 (val == SENSOR_CHAN_GYRO_Y) || (val == SENSOR_CHAN_GYRO_Z))

enum bno055_sensor_trigger_type {
	BNO055_SENSOR_TRIG_HIGH_RATE = SENSOR_TRIG_PRIV_START + 0,
	BNO055_SENSOR_TRIG_HIGH_G = SENSOR_TRIG_PRIV_START + 1
};

#if defined(CONFIG_BNO055_TRIGGER) && DT_ANY_INST_HAS_PROP_STATUS_OKAY(irq_gpios)
#define BNO055_USE_IRQ 1
#endif

#endif /* ZEPHYR_DRIVERS_SENSOR_BNO055_H_ */
