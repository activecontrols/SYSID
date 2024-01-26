#include <Arduino.h>

float weightsA1 = 0.0084914244;
float weightsA2 = -0.0182792591;
float weightsA3 = 0.0115852731;
float weightsA4 = 0.0051548949;
const float TORQUE_CALIBRATION_SLOPE = 0.0108;
const float TORQUE_CALIBRATION_OFFSET = -7.7948;

float measurement1;
float measurement2;
float measurement3;
float measurement4;
float measurement5;

int torqueSensor = A4;
int thrustSensor1 = A0;
int thrustSensor2 = A3;
int thrustSensor3 = A1;
int thrustSensor4 = A2;
float temp1 = 0;
float temp2 = 0;
float temp3 = 0;
float temp4 = 0;
float temp5 = 0;

float currentForce = 1e3;
float forcenm1 = 1e6;
float forcenm2;
float currentAvg;
float lastAvg;
float forceConvergenceThreshold = 0.01;

float currentTorque = 1e3;
float torquenm1 = 1e6;
float torquenm2;
float currentTorqueAvg;
float lastTorqueAvg;
float torqueConvergenceThreshold = 0.01;

float tareWeight = 0;
float tareTorque = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(torqueSensor, INPUT);
  pinMode(thrustSensor1, INPUT);
  pinMode(thrustSensor2, INPUT);
  pinMode(thrustSensor3, INPUT);
  pinMode(thrustSensor4, INPUT);

  tareWeight = getForce();
  tareTorque = getTorque();
}

void loop() {
  // put your main code here, to run repeatedly:
  float force = getForce();
  float torque = getTorque();

  Serial.print(force, 3); Serial.print(","); Serial.println(torque, 3);
  Serial.flush();
  delay(10);
}

float getTorque() {
  float force;
  float moment_arm = 0.1158875;

  force = analogRead(torqueSensor) * TORQUE_CALIBRATION_SLOPE - TORQUE_CALIBRATION_OFFSET;

  return (force * moment_arm) - tareTorque;
}

float getForce() {
  float mass;
  for (int i = 0; i < 10; i++) {
    measurement1 = analogRead(thrustSensor1) * weightsA1;
    temp1 = temp1 + measurement1;

    measurement2 = analogRead(thrustSensor2) * weightsA2;
    temp2 = temp2 + measurement2;

    measurement3 = analogRead(thrustSensor3) * weightsA3;
    temp3 = temp3 + measurement3;

    measurement4 = analogRead(thrustSensor4) * weightsA4;
    temp4 = temp4 + measurement4;
  }
  temp1 = temp1 / 10;
  temp2 = temp2 / 10;
  temp3 = temp3 / 10;
  temp4 = temp4 / 10;
  mass = temp1 + temp2 + temp3 + temp4;
  return (mass * 9.81) - tareWeight;
}
