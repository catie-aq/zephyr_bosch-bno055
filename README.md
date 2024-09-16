# Bosch Sensortec GmbH BNO055 Sensor Driver

Bosch Sensortec GmbH [BNO055](https://www.bosch-sensortec.com/products/smart-sensor-systems/bno055/) 9-axis absolute orientation sensor driver for Zephyr OS.

Based on [BNO055 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf).
Implemented feature:

* [X] BNO055 Operating mode configuration (sensor_attr)
* [X] BNO055 Power mode configuration (sensor_attr)
* [X] Xtal property configuration (DeviceTree)
* [X] Kconfig custom sensors configuration (Accelerometer, Magnetometer, Gyroscope)
* [X] Sample fetch interface (according to BNO055 Operating mode)
* [X] Accelerator Sensor channels getter
* [X] Magnetometer Sensor channels getter
* [X] Gyroscope Sensor channels getter
* [X] Euler Fusion channels getter
* [X] Quaternion Fusion channels getter
* [X] Linear Acceleration Fusion channels getter
* [X] Gravity Fusion channels getter
* [X] Calibration Status getter
* [ ] Axis remapping configuration (DeviceTree)
* [ ] Interrupt configuration (attr + DeviceTree)
* [ ] Interrupt configuration DRDY (attr + DeviceTree)
* [ ] Offset configuration (sensor_attr)
* [ ] SIC Matrix support
