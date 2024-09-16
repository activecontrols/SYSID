#include "IMU.h"
#include "SD_stuff.h"

#include <Servo.h>
#include <elapsedMillis.h>

//#include <math.h>
#include <Arduino.h>
#include <FreqMeasure.h>
#include "Encoder.h"

#define serial Serial1

int constantDelay = 20;

Servo vane1;
Servo vane2;
Servo betaServo;
Servo gammaServo;
Servo EDF;
int vane1Pin = 33;
int vane2Pin = 36;
int betaPin = 28;
int gammaPin = 29;
int EDFPin = 8;

float vane_min = -15;
float vane_max = 15;
float alpha1_0 = 140;  // Initial Vane setting in degrees
float alpha2_0 = 140;

float initialBeta = 100;
float initialGamma = 90;

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

// float lastSegmentTime;
int segmentIdx = 0;

void writeConstant(float in, char identifier) {
  if (identifier == 't') {
    int pwm_out = (int) (
            (in - throttle_min) / (throttle_max - throttle_min) // normalize to 0-1
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

String input;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);

  //serial.begin(9600);
  serial.begin(57600);

  FreqMeasure.begin();


  // while (!serial.available()) {}

  int errorCode = initializeIMU();
  while(errorCode != 0) {
    serial.print("Failed to initialize IMU, error code: ");
    serial.print(errorCode);
    serial.println(". Retrying...");
    errorCode = initializeIMU();
    delay(100);
  }

  initializeSD();

  logger::open("spongebob2.bin");

  encoder::encoderSetup();

  vane1.attach(vane1Pin);
  vane2.attach(vane2Pin);

  betaServo.attach(betaPin);
  gammaServo.attach(gammaPin);
  //writing needs to happen write after attaching or servos max out
  betaServo.write(initialBeta);
  gammaServo.write(initialGamma);

  EDF.attach(EDFPin);
  vane1.write(alpha1_0);
  vane2.write(alpha2_0);
  

  delay(15);
  EDF.writeMicroseconds(1020);
  for (byte i = 0; i < arm_tries; i++) {
    EDF.writeMicroseconds(1000);
  }

  delay(15);
  EDF.writeMicroseconds(1020);
  delay(5000);

  writeConstant(0, 't');
  writeConstant(0, 'a');

  input.reserve(80);

  serial.println("I am alive");
  Serial.println("I am alive");

  // lastSegmentTime = millis() / 1000.0;
}


void delayKeepIMU(int ms) {
  // elapsedMillis t = 0;
  // while (t < ms) {
    // digitalWrite(LED_BUILTIN, LOW);
    // serial.println("BEFORE UPDATE");
    updateIMU();
    // serial.println("AFTER UPDATE");
    delay(1);
    // digitalWrite(LED_BUILTIN, HIGH);
  // }
}

#define LOG_INTERVAL 100 // todo: allow changing this value depending on whether we're actually in a segment or not.

void loop() {
  static elapsedMillis tsincelog;
  static elapsedMillis tsincecmd;

  // serial.println(micros()/100);

  static int thrust = 0;
  static int vane = 0;
  static int numseconds = 0;
  static int linear = 0; // 0 for constant, 1 for linear
  static int thrust2 = 0;
  static int vane2 = 0;

  // serial.println(micros());

  if (tsincelog >= LOG_INTERVAL) {
    // vane_command = 10.0; // just for now
    log();
    // logger::close(); // just for now
    // while (true) {} // just for now.
    tsincelog = 0;
  }

  if (readStringUntil(input, '\n')) {
    
    numseconds = thrust = vane = linear = thrust2 = vane2 = 0;
    sscanf(input.c_str(), "%d %d %d %d %d %d", &numseconds, &thrust, &vane, &linear, &thrust2, &vane2);
    if (numseconds >= 10 || numseconds == 0) {
      writeConstant(0, 't');
      writeConstant(0, 'a');
      logger::close();
      serial.println("ESTOPPED, SD CLOSED");
      while (true) {}
    }
    input = "";
    tsincecmd = 0;
  }

  if (tsincecmd > numseconds * 1000) {
    writeConstant(0, 't');
    writeConstant(0, 'a');
  } else {
    if (!linear) {
      writeConstant((float) thrust, 't');
      writeConstant((float) vane, 'a');
    } else {
      writeLinear((float) tsincecmd / (float) (numseconds * 1000), (float) thrust, (float) thrust2, 't');
      writeLinear((float) tsincecmd / (float) (numseconds * 1000), (float) vane, (float) vane2, 'a');
    }
  }
  

  // serial.println(micros());

  delayKeepIMU(constantDelay);
}

bool readStringUntil(String& input, char until_c) {
  while (serial.available()) {
    char c = serial.read();
    input += c;
    if (c == until_c) {
      return true;
    }
  }
  return false;
}

double sum = 0;
int count = 0;

void log() {
  serial.print(throttle_command);
  serial.print(",");
  serial.println(vane_command);

  serial.print(roll);
  serial.print(",");
  serial.print(pitch);
  serial.print(",");
  serial.println(yaw);

  float frequency = 0.0/0.0; // hacky way to get a nan
  if (FreqMeasure.available()) {
    // average several readings together
    sum = sum + FreqMeasure.read();
    if (++count > 10) {
      frequency = 60 / 2 * FreqMeasure.countToFrequency(sum / count);
      sum = 0;
      count = 0;
    }
  }

  logger::Data data = {
    segmentIdx,
    millis() / 1000.0,
    throttle_command,
    vane_command,
    frequency,
    roll,
    pitch,
    yaw,
    accel.acceleration.x,
    accel.acceleration.y,
    accel.acceleration.z,
    gx,
    gy,
    gz,
    mag.magnetic.x,
    mag.magnetic.y,
    mag.magnetic.z,
    encoder::magEncoder1.readAngle(),
    encoder::magEncoder2.readAngle()
  };

  logger::write(&data);
  // serial.flush();
}