#include <Servo.h>
#include <Math.h>
#include <Arduino.h>
#include <FreqMeasure.h>

// SYS-ID CODE

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
bool startFlag = false;

int segment = 0;
float lastSegmentTime;

float throttle_command;
float vane_command;

float newestForce, newestTorque;

float currentForce = 1e3;
float forcenm1 = 1e6;
float forcenm2;
float currentAvg;
float lastAvg;
float forceConvergenceThreshold = 0.03;

float currentTorque = 1e3;
float torquenm1 = 1e6;
float torquenm2;
float currentTorqueAvg;
float lastTorqueAvg;
float torqueConvergenceThreshold = 0.03;

double sum = 0;
int count = 0;

void setup() {
  // put your setup code here, to run once:
  FreqMeasure.begin();
  Serial.begin(9600);
  Serial1.begin(115200);

  while (!Serial.available()) {
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

  newestForce = 0;
  newestTorque = 0;

  delay(15);
  EDF.writeMicroseconds(1020);
  for (byte i = 0; i < arm_tries; i++) {
    EDF.writeMicroseconds(1000);
  }

  delay(15);
  EDF.writeMicroseconds(1020);
  delay(5000);

  lastSegmentTime = millis() / 1000.0;

  Serial.println("Segment\tTime (s)\tThrottle\tVane Angle (deg)\tThrust Force (N)\tTorque (Nm)\tRPM");
}

void loop() {
  delay(constantDelay);
  if (Serial1.available()) {
    String uno_tlm = Serial1.readStringUntil('\n');
    int comma_index = uno_tlm.indexOf(',');
    newestForce = uno_tlm.substring(0, comma_index).toFloat();
    newestTorque = uno_tlm.substring(comma_index + 1, uno_tlm.length()).toFloat();

    if (millis() > 7000) {

      switch (segment) {
        case 0:

          unitStep(lastSegmentTime, 40, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 10) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 1:

          unitStep(lastSegmentTime, 70, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 10) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 2:

          unitStep(lastSegmentTime, 100, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 10) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 3:

          unitStep(lastSegmentTime, 50, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 10) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 4:
          unitStep(lastSegmentTime, 50, 't');
          unitStep(lastSegmentTime, vane_max, '1');
          unitStep(lastSegmentTime, vane_max, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 10) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }
          break;
        case 5:
          unitStep(lastSegmentTime, 50, 't');
          unitStep(lastSegmentTime, vane_min, '1');
          unitStep(lastSegmentTime, vane_min, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 10) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }
          break;
        case 6:
          unitStep(lastSegmentTime, 50, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 10) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }
          break;

        case 7:
          sineInput(millis() / 1000.0, 1.0, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 15) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 8:
          sineInput(millis() / 1000.0, 0.5, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 15) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 9:
          sineInput(millis() / 1000.0, 0.1, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 15) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 10:
          unitStep(lastSegmentTime, 50, 't');
          sineInput(millis() / 1000.0, 1, '1');
          sineInput(millis() / 1000.0, 1, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 15) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 11:
          unitStep(lastSegmentTime, 50, 't');
          sineInput(millis() / 1000.0, 0.5, '1');
          sineInput(millis() / 1000.0, 0.5, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 15) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        case 12:
          unitStep(lastSegmentTime, 50, 't');
          sineInput(millis() / 1000.0, 0.1, '1');
          sineInput(millis() / 1000.0, 0.1, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

          if ((millis() / 1000.0) > lastSegmentTime + 15) {
            segment++;
            lastSegmentTime = millis() / 1000.0;
          }

          break;
        default:
          unitStep(lastSegmentTime, 0, 't');
          unitStep(lastSegmentTime, 0, '1');
          unitStep(lastSegmentTime, 0, '2');

          currentForce = newestForce;
          currentTorque = newestTorque;

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
      Serial.print(currentForce, 5);
      Serial.print(",\t");
      Serial.print(currentTorque, 5);
      Serial.print(",\t");

      if (FreqMeasure.available()) {
        // average several reading together
        sum = sum + FreqMeasure.read();
        count = count + 1;
        if (count > 10) {
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

      Serial.println();
      Serial.flush();
    } else {
      EDF.writeMicroseconds(1020);
      lastSegmentTime = millis() / 1000.0;
    }
  }
}

bool forceConvergence(char id) {
  forcenm2 = forcenm1;
  forcenm1 = currentForce;
  currentForce = newestForce;

  return ((abs(forcenm2 - forcenm1) / forcenm1) < forceConvergenceThreshold) && ((abs(forcenm1 - currentForce) / currentForce) < forceConvergenceThreshold);
}

bool torqueConvergence() {
  torquenm2 = torquenm1;
  torquenm1 = currentTorque;
  currentTorque = newestTorque;

  return ((abs(torquenm2 - torquenm1) / torquenm1) < torqueConvergenceThreshold) && ((abs(torquenm1 - currentTorque) / currentTorque) < torqueConvergenceThreshold);
}

void unitStep(float t, float end, char identifier) {
  if (identifier == 't') {
    int pwm_out = (int)(((end - throttle_min) / (throttle_max - throttle_min)) * (float)(high_endpoint - low_endpoint) + low_endpoint);
    throttle_command = end;
    EDF.writeMicroseconds(pwm_out);
  } else if (identifier == '1') {
    vane1.write(alpha1_0 + (int)end);
    vane_command = end;
  } else {
    vane2.write(alpha2_0 + (int)end);
    vane_command = end;
  }
}

void sineInput(float t, float angularRate, char identifier) {
  if (identifier == 't') {
    int pwm_out = (int)(((50 + 50 * sin((t - lastSegmentTime) * angularRate) - throttle_min) / (throttle_max - throttle_min)) * (float)(high_endpoint - low_endpoint) + low_endpoint);
    throttle_command = 50 + 50 * sin((t - lastSegmentTime) * angularRate);
    EDF.writeMicroseconds(pwm_out);
  } else if (identifier == '1') {
    vane1.write(alpha1_0 + vane_max * sin((t - lastSegmentTime) * angularRate));
    vane_command = vane_max * sin((t - lastSegmentTime) * angularRate);
  } else {
    vane2.write(alpha2_0 + vane_max * sin((t - lastSegmentTime) * angularRate));
    vane_command = vane_max * sin((t - lastSegmentTime) * angularRate);
  }
}