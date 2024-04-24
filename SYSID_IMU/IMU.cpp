#include "Error.h"
#include "IMU.h"
#include "LSM6DS_LIS3MDL.h"

/*
IMU.cpp 
Description: Function definititions for declarations in IMU.h
Author: Vincent Palmerio

*/


Eigen::VectorXd linearAccelVector(3);
float linearAccelX, linearAccelY, linearAccelZ = 0;
Eigen::VectorXd gyroscopeVector(3);
Eigen::VectorXd magnetometerVector(3);
float roll, pitch, yaw = 0;
float gx, gy, gz = 0; //degrees per second on gyro
float qw, qx, qy, qz = 0; //quaternarion

/*
 * You must free the pointer and set it to NULL after using the pointer!
 */
float* getValues() {
  float* values = (float*)malloc(3 * sizeof(float));
  values[0] = roll;
  values[1] = pitch;
  values[2] = yaw;
  return values;
}


//loads a predetermined calibration into the EEPROM
int loadPresetCalibration() {
  //Magnetic Hard Offset
  cal.mag_hardiron[0] = -3.35;
  cal.mag_hardiron[1] = -0.74;
  cal.mag_hardiron[2] = -40.79;

  //Magnetic Soft Offset
  // in uTesla
  cal.mag_softiron[0] = 0.96;
  cal.mag_softiron[1] = 0.02;
  cal.mag_softiron[2] = 0.01;  
  cal.mag_softiron[3] = 0.02;
  cal.mag_softiron[4] = 0.96;
  cal.mag_softiron[5] = 0.0;  
  cal.mag_softiron[6] = 0.01;
  cal.mag_softiron[7] = 0.0;
  cal.mag_softiron[8] = 1.08;  

  //Gyro zero rate offset
  // in Radians/s
  cal.gyro_zerorate[0] = 0.05;
  cal.gyro_zerorate[1] = -0.01;
  cal.gyro_zerorate[2] = -0.01;

  if (cal.saveCalibration()) {
    return FAILED_LOAD_CALIBRATION;
  }

  return NO_ERROR_CODE;
}



int initializeIMU() {
  //Serial.begin(115200);
  //while (!Serial) yield();
  //Serial.println("test");
  for (int i = 0; i < linearAccelVector.size(); i++) {
    linearAccelVector(i) = 0;
  }


  if (!cal.begin()) {
    //Failed to initialize calibration helper
#if defined(ASTRA_FULL_DEBUG) or defined(ASTRA_IMU_DEBUG)
    Serial.print("Failed to initialize calibration helper");
#endif
    return FAILED_CALIBRATION_HELPER_INIT;
  }

  if (!cal.loadCalibration()) {
    //No calibration loaded/found
    static bool triedLoadPresetCalibration = false;
    if (!triedLoadPresetCalibration) {
      if (loadPresetCalibration() == FAILED_LOAD_CALIBRATION) {
        return FAILED_LOAD_CALIBRATION;
      }
    }
#if defined(ASTRA_FULL_DEBUG) or defined(ASTRA_IMU_DEBUG)
    Serial.println("Failed to load calibration");
#endif
  
  }

  if (!init_sensors(&IMU_WIRE)) {
    //Failed to find sensors
#if defined(ASTRA_FULL_DEBUG) or defined(ASTRA_IMU_DEBUG)
    Serial.println("Failed to find sensors");
#endif
    return FAILED_SENSOR_INIT;
  }
  
#if defined(ASTRA_FULL_DEBUG) or defined(ASTRA_IMU_DEBUG)
  accelerometer->printSensorDetails();
  gyroscope->printSensorDetails();
  magnetometer->printSensorDetails();
#endif

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  
  Wire.setClock(400000); // 400KHz

  return NO_ERROR_CODE;
}

//sets varaibles declared in IMU.h based on latest data IMU
int updateIMU() {
  
  // Read the motion sensors
  sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important

  gx = gyro.gyro.x; //* SENSORS_RADS_TO_DPS; //omega x
  gy = gyro.gyro.y; //* SENSORS_RADS_TO_DPS; //omega y
  gz = gyro.gyro.z; //* SENSORS_RADS_TO_DPS; //omega z

  // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  yaw = filter.getYaw();


  //float qw, qx, qy, qz;
  filter.getQuaternion(&qw, &qx, &qy, &qz);


  filter.getLinearAcceleration(&linearAccelX, &linearAccelY, &linearAccelZ); //"a" -  linear acceleration

  linearAccelVector << linearAccelX, linearAccelY, linearAccelZ;
  gyroscopeVector << gx, gy, gz;
  magnetometerVector << mag.magnetic.x, mag.magnetic.y, mag.magnetic.z;

#if defined(ASTRA_FULL_DEBUG) or defined(ASTRA_IMU_DEBUG)

  //Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");

  //Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
  Serial.print("Raw: ");
  Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  Serial.print(gx, 4); Serial.print(", ");
  Serial.print(gy, 4); Serial.print(", ");
  Serial.print(gz, 4); Serial.print(", ");
  Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  Serial.print(mag.magnetic.z, 4); Serial.println("");

  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.println(roll);

  Serial.print("Quaternion: ");
  Serial.print(qw, 4);
  Serial.print(", ");
  Serial.print(qx, 4);
  Serial.print(", ");
  Serial.print(qy, 4);
  Serial.print(", ");
  Serial.println(qz, 4);  
  //Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");

#endif


  return NO_ERROR_CODE;

}

