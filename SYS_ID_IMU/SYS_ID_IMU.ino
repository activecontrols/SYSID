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
int vane1Pin = 28;
int vane2Pin = 33;
int betaPin = 36;
int gammaPin = 29;
int EDFPin = 8;

float vane_min = -15;
float vane_max = 15;
float alpha1_0 = 140;  // Initial Vane setting in degrees
float alpha2_0 = 145;

const int low_endpoint = 1020;   // 0 throttle = 1000
const int high_endpoint = 1980;  // 100 throttle = 2000
float throttle_min = 0;
float throttle_max = 100;

int delta = high_endpoint - low_endpoint;
int arm_tries = 100;

int segment = 0;
float lastSegmentTime;

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
  Serial.begin(9600);

  while (!Serial.available()) {}

  int errorCode = initializeIMU();
  while(errorCode != 0) {
    Serial.print("Failed to initialize IMU, error code: ");
    Serial.print(errorCode);
    Serial.println(". Retrying...");
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
  Serial.println("Segment\tTime (s)\tThrottle\tVane Angle (deg)\tRPM");
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

  switch (segment) {
    case 0:
      writeConstant(15, 'a');
//      writeLinear((millis() / 1000.0 - lastSegmentTime) / 10, 0, 100, 't');
      if ((millis() / 1000.0) > lastSegmentTime + 10) {
        segment++;
        lastSegmentTime = millis() / 1000.0;
      }
      break;
    case 1:
      writeConstant(-15, 'a');
//      writeLinear((millis() / 1000.0 - lastSegmentTime) / 10, 100, 0, 't');
      if ((millis() / 1000.0) > lastSegmentTime + 10) {
        segment++;
        lastSegmentTime = millis() / 1000.0;
      }
      break;
    default:
      writeConstant(0, 't');
      writeConstant(0, 'a');
      break;
  }

  Serial.print(segment);
  Serial.print(",\t");
  Serial.print(millis() / 1000.0);
  Serial.print(",\t");
  Serial.print(throttle_command);
  Serial.print(",\t");
  Serial.print(vane_command);
  Serial.print(",\t");

  if (FreqMeasure.available()) {
    // average several reading together
    sum = sum + FreqMeasure.read();
    if (++count > 10) {
      float frequency = 60 / 2 * FreqMeasure.countToFrequency(sum / count);
      Serial.print(frequency);
      sum = 0;
      count = 0;
    } else {
      Serial.print("NULL");
    }
  } else {
    Serial.print("NULL");
  }

  Serial.print(roll);
  Serial.print(",");
  Serial.print(pitch);
  Serial.print(",");
  Serial.print(yaw);
  Serial.print(",");

  Serial.println();
  Serial.flush();
}
