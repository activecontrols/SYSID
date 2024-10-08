
#include "Error.h"

#include "IMU.h"
#include "LSM6DS_LIS3MDL.h"
#include "MEKF.h"
#include "math.h"

/*
IMU.cpp 
Description: Function definititions for declarations in IMU.h
Author: Vincent Palmerio

*/

//#define ASTRA_IMU_DEBUG

Eigen::VectorXd linearAccelVector(3);
float linearAccelX, linearAccelY, linearAccelZ = 0;
Eigen::VectorXd gyroscopeVector(3);
Eigen::VectorXd magnetometerVector(3);
float roll, pitch, yaw = 0;
float gx, gy, gz = 0; //degrees per second on gyro
float qw, qx, qy, qz = 0; //quaternarion
sensors_event_t accel, gyro, mag;
elapsedMillis filterMillis = 0;
int lastFilterMillis = 0;

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

#if IMU_NUMBER == 1
  //Magnetic Hard Offset
  cal.mag_hardiron[0] = 18.00;
  cal.mag_hardiron[1] = 37.79;
  cal.mag_hardiron[2] = -84.00;

  //Magnetic Soft Offset
  // in uTesla
  cal.mag_softiron[0] = 0.98;
  cal.mag_softiron[1] = 0.05;
  cal.mag_softiron[2] = 0.01;  
  cal.mag_softiron[3] = 0.05;
  cal.mag_softiron[4] = 1.06;
  cal.mag_softiron[5] = 0.0;  
  cal.mag_softiron[6] = 0.01;
  cal.mag_softiron[7] = 0.0;
  cal.mag_softiron[8] = 0.96;  

  //Gyro zero rate offset
  // in Radians/s
  cal.gyro_zerorate[0] = 0.0056;
  cal.gyro_zerorate[1] = -0.00032197;
  cal.gyro_zerorate[2] = 0.000635;
#endif

#if IMU_NUMBER == 2 /* NOT CALIBRATED YET */
  //Magnetic Hard Offset
  cal.mag_hardiron[0] = 0.0;
  cal.mag_hardiron[1] = 0.0;
  cal.mag_hardiron[2] = 0.0;

  //Magnetic Soft Offset
  // in uTesla
  cal.mag_softiron[0] = 0.0;
  cal.mag_softiron[1] = 0.0;
  cal.mag_softiron[2] = 0.0;  
  cal.mag_softiron[3] = 0.0;
  cal.mag_softiron[4] = 0.0;
  cal.mag_softiron[5] = 0.0;  
  cal.mag_softiron[6] = 0.0;
  cal.mag_softiron[7] = 0.0;
  cal.mag_softiron[8] = 0.0;  

  //Gyro zero rate offset
  // in Radians/s
  cal.gyro_zerorate[0] = 0.0;
  cal.gyro_zerorate[1] = 0.0;
  cal.gyro_zerorate[2] = 0.0;
#endif

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
  loadPresetCalibration();
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
  initKalman(Eigen::Quaterniond::Identity(), 0.0, 0.0000194955, 0.000000000000000025032, 0.0000000024616355, 0.0, 0.0000000024616355, 0.0, 0.0003, 0.00003);
  filter.begin(FILTER_UPDATE_RATE_HZ);
  
  Wire.setClock(400000); // 400KHz

  return NO_ERROR_CODE;
}

//sets varaibles declared in IMU.h based on latest data IMU
int updateIMU() {

  // Serial.println("CAN YOU SEE ME");
  
  // Read the motion sensors
  // sensors_event_t accel, gyro, mag;
  accelerometer->getEvent(&accel);
  gyroscope->getEvent(&gyro);
  magnetometer->getEvent(&mag);

  // Serial.println("CAN YOU SEE ME 2");

  cal.calibrate(mag);
  cal.calibrate(accel);
  cal.calibrate(gyro);

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important

  gx = gyro.gyro.x; //omega x
  gy = gyro.gyro.y; //omega y
  gz = gyro.gyro.z; //omega z

  // Update the SensorFusion filter
  // filter.update(gx, gy, gz, 
  //               accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
  //               mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
    // filter.update(gx, gy, gz, 
    //             accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
    //             0,0,0);
  updateKalman(Eigen::Vector3d(gx, gy, gz), Eigen::Vector3d(-accel.acceleration.x/9.8, -accel.acceleration.y/9.8, -accel.acceleration.z/9.8), 
                Eigen::Vector3d(0.0, 0.0, 0.0), (double)(filterMillis - lastFilterMillis)/1000.0);
  lastFilterMillis = filterMillis;
  // Serial.println("RAW:  ");
  // Serial.print(lsm6ds.rawAccX);
  // Serial.print(",");
  // Serial.print(lsm6ds.rawAccY);
  // Serial.print(",");
  // Serial.print(lsm6ds.rawAccZ);
  // Serial.print(",");
  // Serial.print(lsm6ds.rawGyroX);
  // Serial.print(",");
  // Serial.print(lsm6ds.rawGyroY);
  // Serial.print(",");
  // Serial.print(lsm6ds.rawGyroZ);
  // Serial.print(",");
  // Serial.print(lis3mdl.x);
  // Serial.print(",");
  // Serial.print(lis3mdl.y);
  // Serial.print(",");
  // Serial.print(lis3mdl.z);
  // Serial.print(",");
  // filter.update(lsm6ds.rawGyroX, lsm6ds.rawGyroY, lsm6ds.rawGyroZ,
  //               lsm6ds.rawAccX, lsm6ds.rawAccY, lsm6ds.rawAccZ,
  //               lis3mdl.x, lis3mdl.y, lis3mdl.z
  // );
  
  // print the heading, pitch and roll

  //roll = filter.getRoll();
  //pitch = filter.getPitch();
  //yaw = filter.getYaw();

  // gx = lsm6ds.rawGyroX;
  // gy = lsm6ds.rawGyroY;
  // gz = lsm6ds.rawGyroZ;

  //float qw, qx, qy, qz;
  qw = estimate.w();
  qx = estimate.x();
  qy = estimate.y();
  qz = estimate.z();

  Eigen::Vector3d v = quatToEuler(estimate);
  roll = v.x() * SENSORS_RADS_TO_DPS;
  pitch = v.y() * SENSORS_RADS_TO_DPS;
  yaw = v.z() * SENSORS_RADS_TO_DPS;
  //filter.getQuaternion(&qw, &qx, &qy, &qz);


  filter.getLinearAcceleration(&linearAccelX, &linearAccelY, &linearAccelZ); //"a" -  linear acceleration

  linearAccelVector << linearAccelX, linearAccelY, linearAccelZ;
  gyroscopeVector << gx, gy, gz;
  magnetometerVector << mag.magnetic.x, mag.magnetic.y, mag.magnetic.z;

  // Serial.println();
  // Serial.print("Orientation: ");
  // Serial.print(roll);
  // Serial.print(", ");
  // Serial.print(pitch);
  // Serial.print(", ");
  // Serial.println(yaw);

#if defined(ASTRA_FULL_DEBUG) or defined(ASTRA_IMU_DEBUG)

  //Serial.print("I2C took "); Serial.print(millis()-timestamp); Serial.println(" ms");;
  //Serial.print("Update took "); Serial.print(millis()-timestamp); Serial.println(" ms");
  // Serial.print("Raw: ");
  // Serial.print(accel.acceleration.x, 4); Serial.print(", ");
  // Serial.print(accel.acceleration.y, 4); Serial.print(", ");
  // Serial.print(accel.acceleration.z, 4); Serial.print(", ");
  // Serial.println();
  // Serial.print(gx, 4); Serial.print(", ");
  // Serial.print(gy, 4); Serial.print(", ");
  // Serial.print(gz, 4); Serial.print(", ");
  // Serial.println();
  // Serial.print(mag.magnetic.x, 4); Serial.print(", ");
  // Serial.print(mag.magnetic.y, 4); Serial.print(", ");
  // Serial1.print(mag.magnetic.z, 4); Serial1.println("\n");

  

  // Serial.print("Quaternion: ");
  // Serial.print(qw, 4);
  // Serial.print(", ");
  // Serial.print(qx, 4);
  // Serial.print(", ");
  // Serial.print(qy, 4);
  // Serial.print(", ");
  // Serial.println(qz, 4);  
  //Serial.print("Took "); Serial.print(millis()-timestamp); Serial.println(" ms");
  //delay(50);
#endif


  return NO_ERROR_CODE;

}

Eigen::Vector3d quatToEuler(Eigen::Quaterniond q){
  Eigen::Vector3d v(0.0, 0.0, 0.0);
  v.x() = atan2(2*(q.w()*q.x() + q.y()*q.z()), 1 - 2*(q.x()*q.x() + q.y()*q.y()));
  v.y() = asin(2*(q.w()*q.y() - q.z()*q.x()));
  v.z() = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(q.y()*q.y() + q.z()*q.z()));
  return v;
}

