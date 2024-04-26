#include "IMU.h"

#include <Servo.h>

//#include <math.h>
#include <Arduino.h>
#include <FreqMeasure.h>

int constantDelay = 20;

Servo vane1;
Servo vane2;
Servo betaServo;
Servo gammaServo;
Servo EDF;
int vane1Pin = 33;
int vane2Pin = 36;
int betaPin = 39;
int gammaPin = 29;
int EDFPin = 8;

float vane_min = -15;
float vane_max = 15;
float alpha1_0 = 140;  // Initial Vane setting in degrees
float alpha2_0 = 140;

const int low_endpoint = 1020;   // 0 throttle = 1000
const int high_endpoint = 1980;  // 100 throttle = 2000
float throttle_min = 0;
float throttle_max = 100;

int delta = high_endpoint - low_endpoint;
int arm_tries = 100;

float throttle_command;
float vane_command;

float forcenm1 = 1e6;
float forcenm2;
float forceConvergenceThreshold = 0.03;

float torquenm1 = 1e6;
float torquenm2;
float torqueConvergenceThreshold = 0.03;

double sum = 0;
int count = 0;

float lastSegmentTime;
int segmentIdx = 0;

void writeConstant(float in, char identifier) {
  if (identifier == 't') {
    int pwm_out = (int) (
            in / (throttle_max - throttle_min) // normalize to 0-1
              * (float)(high_endpoint - low_endpoint) + low_endpoint // scale to endpoints
            );
    throttle_command = in;
    EDF.writeMicroseconds(pwm_out);
  } else if (identifier == 'a') {
    vane1.write(alpha1_0 + (int)in);
    vane2.write(alpha2_0 + (int)in);
    vane_command = in;
  }
}

// t in range 0 to 1
void writeLinear(float t, float start, float end, char identifier) {
  float input = start + (end - start) * t;
  switch (identifier) {
    case 't':
      writeConstant(input, 't');
      break;
    case 'a':
      writeConstant(input, 'a');
      break;
    default:
        break;
  }
}

void setup() {
  FreqMeasure.begin();
  Serial1.begin(57600);

  while (!Serial1.available()) {}

  int errorCode = initializeIMU();
  while(errorCode != 0) {
    Serial1.print("Failed to initialize IMU, error code: ");
    Serial1.print(errorCode);
    Serial1.println(". Retrying...");
    errorCode = initializeIMU();
  }

  vane1.attach(vane1Pin);
  vane2.attach(vane2Pin);
  betaServo.attach(betaPin);
  gammaServo.attach(gammaPin);
  EDF.attach(EDFPin);
  vane1.write(alpha1_0);
  vane2.write(alpha2_0);
  betaServo.write(90);
  gammaServo.write(100);

  delay(15);
  EDF.writeMicroseconds(1020);
  for (byte i = 0; i < arm_tries; i++) {
    EDF.writeMicroseconds(1000);
  }

  delay(15);
  EDF.writeMicroseconds(1020);
  delay(5000);

  lastSegmentTime = millis() / 1000.0;
  Serial1.println("segment,time,throttle,vane,rpm,roll,pitch,yaw,accelx,accely,accelz,gx,gy,gz,magx,magy,magz");
}

void loop() {
  delay(constantDelay);
  updateIMU(); 

  if (millis() <= 7000) { // do nothing for 7 seconds
    writeConstant(0, 't');
    writeConstant(0, 'a');
    lastSegmentTime = millis() / 1000.0;
    return;
  }

  // actual control input
  
  control();

  // logging:

  Serial1.print(segmentIdx);
  Serial1.print(",");
  Serial1.print(millis() / 1000.0);
  Serial1.print(",");
  Serial1.print(throttle_command);
  Serial1.print(",");
  Serial1.print(vane_command);
  Serial1.print(",");

  if (FreqMeasure.available()) {
    // average several readings together
    sum = sum + FreqMeasure.read();
    if (++count > 10) {
      float frequency = 60 / 2 * FreqMeasure.countToFrequency(sum / count);
      Serial1.print(frequency); Serial1.print(",");
      sum = 0;
      count = 0;
    } else {
      Serial1.print("nan,");
    }
  } else {
    Serial1.print("nan,");
  }

  // filtered values
  Serial1.print(roll);  Serial1.print(",");
  Serial1.print(pitch); Serial1.print(",");
  Serial1.print(yaw);   Serial1.print(",");

  // raw values
  Serial1.print(accel.acceleration.x, 4); Serial1.print(",");
  Serial1.print(accel.acceleration.y, 4); Serial1.print(",");
  Serial1.print(accel.acceleration.z, 4); Serial1.print(",");
  Serial1.print(gx, 4); Serial1.print(",");
  Serial1.print(gy, 4); Serial1.print(",");
  Serial1.print(gz, 4); Serial1.print(",");
  Serial1.print(mag.magnetic.x, 4); Serial1.print(",");
  Serial1.print(mag.magnetic.y, 4); Serial1.print(",");
  Serial1.print(mag.magnetic.z, 4);

  Serial1.println();
  Serial1.flush();
}

int numSegments = 4;
float throttleVals[] = {20, 40, 60, 80};
float vaneVals[]     = {-15, -15, -15, -15};
float secondsPerSegment = 2;
float delayBetween = 2;

void control() {

  if (segmentIdx >= numSegments) {
    writeConstant(0, 't');
    writeConstant(0, 'a');
    delay(100 * 1000); // dont wanna spam output once test is done.
  }

  float secondsSince = millis() / 1000.0 - lastSegmentTime;
  if (secondsSince > secondsPerSegment) {
    delay(delayBetween * 1000.0);
    lastSegmentTime = millis() / 1000.0;
    segmentIdx++;
    return;
  }

  writeConstant(throttleVals[segmentIdx], 't');
  writeConstant(vaneVals[segmentIdx], 'a');
}
