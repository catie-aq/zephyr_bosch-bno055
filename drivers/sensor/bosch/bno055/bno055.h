/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_BNO055_BNO055_H_
#define ZEPHYR_DRIVERS_SENSOR_BNO055_BNO055_H_

/* Registers (PAGE 0) */
#define BNO055_REGISTER_PAGE_ID	0x07

// Identification Registers
#define BNO055_REGISTER_CHIP_ID	0x00
#define BNO055_REGISTER_ACC_ID	0x01
#define BNO055_REGISTER_MAG_ID	0x02
#define BNO055_REGISTER_GYR_ID	0x03

// Revision Registers
#define BNO055_REGISTER_SOFTWARE_REV	    BNO055_REGISTER_SOFTWARE_REV_LSB // 2 Octets
#define BNO055_REGISTER_SOFTWARE_REV_LSB	0x04
#define BNO055_REGISTER_SOFTWARE_REV_MSB	0x05
#define BNO055_REGISTER_BOOTLOADER_REV      0x06

// Accelerometer Registers
#define BNO055_REGISTER_ACC_DATA_X_LSB	0x08
#define BNO055_REGISTER_ACC_DATA_X_MSB	0x09
#define BNO055_REGISTER_ACC_DATA_Y_LSB	0x0A
#define BNO055_REGISTER_ACC_DATA_Y_MSB	0x0B
#define BNO055_REGISTER_ACC_DATA_Z_LSB	0x0C
#define BNO055_REGISTER_ACC_DATA_Z_MSB	0x0D

// Magnetometer Registers
#define BNO055_REGISTER_MAG_DATA_X_LSB	0x0E
#define BNO055_REGISTER_MAG_DATA_X_MSB	0x0F
#define BNO055_REGISTER_MAG_DATA_Y_LSB	0x10
#define BNO055_REGISTER_MAG_DATA_Y_MSB	0x11
#define BNO055_REGISTER_MAG_DATA_Z_LSB	0x12
#define BNO055_REGISTER_MAG_DATA_Z_MSB	0x13

// Gyroscope Registers
#define BNO055_REGISTER_GYR_DATA_X_LSB	0x14
#define BNO055_REGISTER_GYR_DATA_X_MSB	0x15
#define BNO055_REGISTER_GYR_DATA_Y_LSB	0x16
#define BNO055_REGISTER_GYR_DATA_Y_MSB	0x17
#define BNO055_REGISTER_GYR_DATA_Z_LSB	0x18
#define BNO055_REGISTER_GYR_DATA_Z_MSB	0x19

// Euler Registers
#define BNO055_REGISTER_EUL_HEADING_LSB	0x1A
#define BNO055_REGISTER_EUL_HEADING_MSB	0x1B
#define BNO055_REGISTER_EUL_ROLL_LSB	0x1C
#define BNO055_REGISTER_EUL_ROLL_MSB	0x1D
#define BNO055_REGISTER_EUL_PITCH_LSB	0x1E
#define BNO055_REGISTER_EUL_PITCH_MSB	0x1F

// Quaternion Registers
#define BNO055_REGISTER_QUA_DATA_W_LSB	0x20
#define BNO055_REGISTER_QUA_DATA_W_MSB	0x21
#define BNO055_REGISTER_QUA_DATA_X_LSB	0x22
#define BNO055_REGISTER_QUA_DATA_X_MSB	0x23
#define BNO055_REGISTER_QUA_DATA_Y_LSB	0x24
#define BNO055_REGISTER_QUA_DATA_Y_MSB	0x25
#define BNO055_REGISTER_QUA_DATA_Z_LSB	0x26
#define BNO055_REGISTER_QUA_DATA_Z_MSB	0x27

// Linear Acceleration Registers
#define BNO055_REGISTER_LIA_DATA_X_LSB	0x28
#define BNO055_REGISTER_LIA_DATA_X_MSB	0x29
#define BNO055_REGISTER_LIA_DATA_Y_LSB	0x2A
#define BNO055_REGISTER_LIA_DATA_Y_MSB	0x2B
#define BNO055_REGISTER_LIA_DATA_Z_LSB	0x2C
#define BNO055_REGISTER_LIA_DATA_Z_MSB	0x2D

// Gravity Vector Registers
#define BNO055_REGISTER_GRV_DATA_X_LSB	0x2E
#define BNO055_REGISTER_GRV_DATA_X_MSB	0x2F
#define BNO055_REGISTER_GRV_DATA_Y_LSB	0x30
#define BNO055_REGISTER_GRV_DATA_Y_MSB	0x31
#define BNO055_REGISTER_GRV_DATA_Z_LSB	0x32
#define BNO055_REGISTER_GRV_DATA_Z_MSB	0x33

#define BNO055_REGISTER_TEMP                0x34
#define BNO055_REGISTER_CALIBRATION_STATUS  0x35 // [SYS(2)|GYR(2)|MAG(2)|ACC(2)]
#define BNO055_REGISTER_SELF_TEST_RESULT    0x36
#define BNO055_REGISTER_INT_STA             0x37
#define BNO055_REGISTER_SYS_CLK_STATUS      0x38
#define BNO055_REGISTER_SYS_STATUS          0x39
#define BNO055_REGISTER_SYS_ERROR           0x3A

#define BNO055_REGISTER_UNIT_SELECT     0x3B
#define BNO055_REGISTER_OPERATION_MODE  0x3D
#define BNO055_REGISTER_POWER_MODE      0x3E
#define BNO055_REGISTER_SYS_TRIGGER     0x3F
#define BNO055_REGISTER_TEMP_SOURCE     0x40
#define BNO055_REGISTER_AXIS_MAP_CONF   0x41
#define BNO055_REGISTER_AXIS_MAP_SIGN   0x42

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
#define BNO055_REGISTER_ACC_OFFSET_X_LSB    0x55
#define BNO055_REGISTER_ACC_OFFSET_X_MSB	0x56
#define BNO055_REGISTER_ACC_OFFSET_Y_LSB	0x57
#define BNO055_REGISTER_ACC_OFFSET_Y_MSB	0x58
#define BNO055_REGISTER_ACC_OFFSET_Z_LSB	0x59
#define BNO055_REGISTER_ACC_OFFSET_Z_MSB	0x5A

// Magnetometer Offset Registers
#define BNO055_REGISTER_MAG_OFFSET_X_LSB	0x5B
#define BNO055_REGISTER_MAG_OFFSET_X_MSB	0x5C
#define BNO055_REGISTER_MAG_OFFSET_Y_LSB	0x5D
#define BNO055_REGISTER_MAG_OFFSET_Y_MSB	0x5E
#define BNO055_REGISTER_MAG_OFFSET_Z_LSB	0x5F
#define BNO055_REGISTER_MAG_OFFSET_Z_MSB	0x60

// Gyroscope Offset Registers
#define BNO055_REGISTER_GYR_OFFSET_X_LSB	0x61
#define BNO055_REGISTER_GYR_OFFSET_X_MSB	0x62
#define BNO055_REGISTER_GYR_OFFSET_Y_LSB	0x63
#define BNO055_REGISTER_GYR_OFFSET_Y_MSB	0x64
#define BNO055_REGISTER_GYR_OFFSET_Z_LSB	0x65
#define BNO055_REGISTER_GYR_OFFSET_Z_MSB	0x66

#define BNO055_REGISTER_ACC_RADIUS_LSB  0x67
#define BNO055_REGISTER_ACC_RADIUS_MSB  0x68
#define BNO055_REGISTER_MAG_RADIUS_LSB  0x69
#define BNO055_REGISTER_MAG_RADIUS_MSB  0x6A

/* Registers (PAGE 1) */
#define BNO055_REGISTER_ACC_CONFIG          0x08
#define BNO055_REGISTER_MAG_CONFIG          0x09
#define BNO055_REGISTER_GYR_CONFIG_0        0x0A
#define BNO055_REGISTER_GYR_CONFIG_1        0x0B
#define BNO055_REGISTER_ACC_SLEEP_CONFIG    0x0C
#define BNO055_REGISTER_GYR_SLEEP_CONFIG    0x0D

#define BNO055_REGISTER_INT_MASK    0x0F
#define BNO055_REGISTER_INT_ENABLE  0x10

#define BNO055_REGISTER_ACC_ANY_MOTION_THRESHOLD    0x11
#define BNO055_REGISTER_ACC_INT_SETTINGS            0x12
#define BNO055_REGISTER_ACC_HIGH_GRAVITY_DURATION   0x13
#define BNO055_REGISTER_ACC_HIGH_GRAVITY_THRESHOLD  0x14
#define BNO055_REGISTER_ACC_NO_MOTION_THRESHOLD     0x15
#define BNO055_REGISTER_ACC_NO_MOTION_SET           0x16

#define BNO055_REGISTER_GYR_INT_SETTINGS            0x17
#define BNO055_REGISTER_GYR_HIGH_RATE_X_SET         0x18
#define BNO055_REGISTER_GYR_DURATION_X              0x19
#define BNO055_REGISTER_GYR_HIGH_RATE_Y_SET         0x1A
#define BNO055_REGISTER_GYR_DURATION_Y              0x1B
#define BNO055_REGISTER_GYR_HIGH_RATE_Z_SET         0x1C
#define BNO055_REGISTER_GYR_DURATION_Z              0x1D
#define BNO055_REGISTER_GYR_ANY_MOTION_THRESHOLD    0x1E
#define BNO055_REGISTER_GYR_ANY_MOTION_SET          0x1F

#define BNO055_CHIP_ID 0xA0

/* Command Register */
#define BNO055_COMMAND_RESET 0x20

/* Timings */
#define BNO055_TIMING_STARTUP   500 // 400ms
#define BNO055_TIMING_RESET_CONFIG  800 // 650ms


/* BNO055 Configuration */
enum PageId {
    PAGE_ZERO    = 0x00,
    PAGE_ONE     = 0x01
};

enum PowerMode {
    NORMAL      = 0x00,
    LOW_POWER   = 0x01,
    SUSPEND     = 0x02,
    INVALID     = 0x03
};

enum OperatingMode {
    CONFIG_MODE     = 0x00,
    ACC_ONLY        = 0x01,
    MAG_ONLY        = 0x02,
    GYRO_ONLY       = 0x03,
    ACC_MAG         = 0x04,
    ACC_GYRO        = 0x05,
    MAGG_GYRO       = 0x06,
    ACC_MAG_GYRO    = 0x07,
    IMU             = 0x08,
    COMPASS         = 0x09,
    M4G             = 0x0A,
    NDOF_FMC_OFF    = 0x0B,
    NDOF            = 0x0C
};

#define FROM_CONFIG_MODE_SWITCHING_TIME 10 // 7 ms
#define FROM_ANY_MODE_SWITCHING_TIME 20 // 19 ms

/* Accelerometer Configuration */
enum ACC_GRange {
    ACC_2G  = 0x00,
    ACC_4G  = 0x01,
    ACC_8G  = 0x02,
    ACC_16G = 0x03
};

enum ACC_Bandwidth {
    ACC_8Hz     = 0x00,
    ACC_16Hz    = 0x01,
    ACC_31Hz    = 0x02,
    ACC_62Hz    = 0x03,
    ACC_125Hz   = 0x04,
    ACC_250Hz   = 0x05,
    ACC_500Hz   = 0x06,
    ACC_1000Hz  = 0x07
};

enum ACC_OperatingMode {
    ACC_NORMAL          = 0x00,
    ACC_SUSPEND         = 0x01,
    ACC_LOW_POWER_1     = 0x02,
    ACC_STANDBY         = 0x03,
    ACC_LOW_POWER_2     = 0x04,
    ACC_DEEP_SUSPEND    = 0x05
};

struct acc_config { // BNO055_REGISTER_ACC_CONFIG
    enum ACC_GRange         range;
    enum ACC_Bandwidth      bandwidth;
    enum ACC_OperatingMode  mode;
};

/* Gyroscope Configuration */
enum GYR_GRange {
    GYR_2000DPS = 0x00,
    GYR_1000DPS = 0x01,
    GYR_500DPS  = 0x02,
    GYR_250DPS  = 0x03,
    GYR_125DPS  = 0x04
};

enum GYR_Bandwidth {
    GYR_523Hz   = 0x00,
    GYR_230Hz   = 0x01,
    GYR_116Hz   = 0x02,
    GYR_47Hz    = 0x03,
    GYR_23Hz    = 0x04,
    GYR_12Hz    = 0x05,
    GYR_64Hz    = 0x06,
    GYR_32Hz    = 0x07
};

enum GYR_OperatingMode {
    GYR_NORMAL              = 0x00,
    GYR_FAST_POWER_UP       = 0x01,
    GYR_DEEP_SUSPEND        = 0x02,
    GYR_SUSPEND             = 0x03,
    GYR_ADVANCED_POWERSAVE  = 0x04
};

struct gyr_config {
    enum GYR_GRange         range; // BNO055_REGISTER_GYR_CONFIG_0
    enum GYR_Bandwidth      bandwidth; // BNO055_REGISTER_GYR_CONFIG_0
    enum GYR_OperatingMode  mode; // BNO055_REGISTER_GYR_CONFIG_1
};

/* Magnetometer Configuration */
enum MAG_Rate {
    MAG_2Hz     = 0x00,
    MAG_6Hz     = 0x01,
    MAG_8Hz     = 0x02,
    MAG_10Hz    = 0x03,
    MAG_15Hz    = 0x04,
    MAG_20Hz    = 0x05,
    MAG_25Hz    = 0x06,
    MAG_30Hz    = 0x07
};

enum MAG_OperatingMode {
    MAG_LOW_POWER           = 0x00,
    MAG_REGULAR             = 0x01,
    MAG_ENHANCED_REGULAR    = 0x02,
    MAG_HIGH_ACCURACY       = 0x03
};

enum MAG_PowerMode {
    MAG_NORMAL      = 0x00,
    MAG_SLEEP       = 0x01,
    MAG_SUSPEND     = 0x02,
    MAG_FORCE_MODE  = 0x03
};

struct mag_config { // BNO055_REGISTER_MAG_CONFIG
    enum MAG_Rate           rate;
    enum MAG_OperatingMode  mode;
    enum MAG_PowerMode      power;
};

/* Unit Selection Configuration */
enum Orientation { // Pitch relative
    WINDOWS = 0x00, // clockwise
    ANDROID = 0x01  // anti-clockwise (trigonometric)
};

enum TemperatureUnit {
    CELSIUS     = 0x00,
    FAHRENHEIT  = 0x01
};

enum EulerUnit {
    DEGREES = 0x00,
    RADIANS = 0x01
};

static const uint8_t EulerConvertion[2] = {16, 900}; // DEGREES | RADIANS

enum RotationUnit {
    DPS = 0x00,
    RPS = 0x01
};

static const uint8_t RotationConvertion[2] = {16, 900}; // DPS | RPS

enum AccelerationUnit {
    MS_2    = 0x00,
    MG      = 0x01
};

static const uint8_t AccelerationConvertion[2] = {100, 1}; // MS_2 | MG

struct unit_config {
    enum Orientation        orientation;
    enum TemperatureUnit    temp;
    enum EulerUnit          euler;
    enum RotationUnit       rotation;
    enum AccelerationUnit   acceleration;
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

#endif /* ZEPHYR_DRIVERS_SENSOR_BNO055_BNO055_H_ */
