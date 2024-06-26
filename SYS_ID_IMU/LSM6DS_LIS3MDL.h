
/*
LSM6DS_LIS3MDL.h
Description: Header file for sensor-setup function definitions
Author: Vincent Palmerio, (Original: https://github.com/adafruit/Adafruit_AHRS/blob/master/examples/calibrated_orientation/calibrated_orientation.ino)


Last updated: 11/4/2023

*/

#ifndef LSM6DS_LIS3MDL
#define LSM6DS_LIS3MDL

#include <Adafruit_LIS3MDL.h>
Adafruit_LIS3MDL lis3mdl;

#include <Adafruit_LSM6DSOX.h>
Adafruit_LSM6DSOX lsm6ds;

#include <Wire.h>

Adafruit_Sensor* accelerometer;
Adafruit_Sensor* gyroscope;
Adafruit_LIS3MDL* magnetometer;

bool init_sensors(TwoWire *i2c_wire) {

  //See https://www.adafruit.com/product/4517 for possible I2C addresses for each device
  if (!lsm6ds.begin_I2C() && !lsm6ds.begin_I2C((uint8_t) 0x6A, i2c_wire, 0L) && !lsm6ds.begin_I2C((uint8_t) 0x6B, i2c_wire, 0L)) {
    return false;
  }
  if (!lis3mdl.begin_I2C((uint8_t) 0x1C, i2c_wire)) {
    lis3mdl = *(new Adafruit_LIS3MDL);
    if (!lis3mdl.begin_I2C((uint8_t) 0x1E, i2c_wire)) {
      return false;
    }
  }

  accelerometer = lsm6ds.getAccelerometerSensor();
  gyroscope = lsm6ds.getGyroSensor();
  magnetometer = &lis3mdl;

  return true;
}

void setup_sensors(void) {
  // set lowest range
  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lis3mdl.setRange(LIS3MDL_RANGE_12_GAUSS);

  // set slightly above refresh rate
  lsm6ds.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_5_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
}

#endif