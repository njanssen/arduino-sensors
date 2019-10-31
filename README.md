# Instant Composition // Arduino Sensors

Arduino sensors for Instant Composition

## FeatherIMU 

Sensor is based on an [Adafruit Feather M0 WiFi](https://learn.adafruit.com/adafruit-feather-m0-wifi-atwinc1500/overview) connected to an [Adafruit BNO-055 absolute orientation sensor](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview) via I2C. Calibration code for the BNO-055 sensor can be generated with the BNO055Calibration sketch.

This sensor: 
* Reads IMU data (accel, linearaccel, gyroscope, magnetometer) and IMU sensor fusion data (euler, quaternion) from the BNO-055 sensor
* Sends this IMU data over UDP/OSC port 9000 in OSC Bundles using a WiFi connection.

## Nano33IMU

Sensor is based on an [Arduino Nano 33 BLE Sense](https://www.arduino.cc/en/Guide/NANO33BLESense).

This sensor:
* Reads IMU data (accel, gyroscope, magnetometer) from the on-board LSM9DS1 sensor
* Acts as a BLE peripheral, BLE centrals can connect to the Arduino and read the IMU data over the advertised services.
