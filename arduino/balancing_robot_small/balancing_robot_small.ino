#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <KalmanFilter.h>
#include <PID_v1.h>
#include <Filters.h>
#include <Encoder.h>

// Defining motor and encoder pins
const int EN1 = 10; const int EN2 = 9;
const int DIR1 = 12; const int DIR2 = 7;

#define S1A  2
#define S2A  5
#define S1B  3
#define S2B  6

Encoder leftEncoder(S1A, S1B);
Encoder rightEncoder(S2A, S2B);

unsigned long oldPositionLeft = -999;
unsigned long oldPositionRight = -999;
// filters out changes faster that 1.6 Hz.
float filterFrequency = 1.6;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

// Use the LSM9DS1 class to create an object.
LSM9DS1 imu;

// Use the KalmanFilter class to create an object.
KalmanFilter kalmanFilter(2, 2, 0.1);

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW

// PID Controller Angle
#define PID_ANGLE_OUTPUT_LOW 0
#define PID_ANGLE_OUTPUT_HIGH 255
double kp = 5.0; double ki = 0.0; double kd = 0.0;
double actualValueAngle = 0.0; double setValueAngle = 90.0; double pidAngleOutput = 0.0;
PID pid(&actualValueAngle, &pidAngleOutput, &setValueAngle, kp, ki, kd, DIRECT);


// Constants representing the states in the state machine
const int S_INIT = 0;
const int S_RUNNING = 1;
int currentState = S_INIT; // A variable holding the current state

void setup() {
  Serial.begin(19200);
  initSensors();
  initMotorShield();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(5);
  pid.SetOutputLimits(PID_ANGLE_OUTPUT_LOW, PID_ANGLE_OUTPUT_HIGH);
}

void loop() {
  updateSensors();
  double angle = getAngle(imu.ay, imu.az);
  kalmanFilter.in(angle);
  double dd = kalmanFilter.out();
  lowpassFilter.input(dd);
  actualValueAngle = lowpassFilter.output();
  actualValueAngle = actualValueAngle * -1;
  //Serial.println(actualValueAngle);

  switch (currentState) {
    case S_INIT:
      if (abs(actualValueAngle - setValueAngle) < 50) {
        turnOnMotors();
        changeState(S_RUNNING);
      }
      break;
    case S_RUNNING:
      if (pid.Compute()) {
        runMotors(pidAngleOutput);
      }
      if (abs(actualValueAngle - setValueAngle) > 50) {
        turnOffMotors();
        changeState(S_INIT);
      }
      break;
  }
  //printData();
}

void updateGyroReadings() {
  if (imu.gyroAvailable()) {
    imu.readGyro();
  }
}

void updateAccelReadings() {
  if (imu.accelAvailable()) {
    imu.readAccel();
  }
}

double getAngle(float ay, float az) {
  double angle = atan2(ay, az);
  angle *= 180.0 / PI;
  return angle;
}

void initSensors() {
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.begin();
}

void initMotorShield() {
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);

  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
}

void updateSensors() {
  updateGyroReadings();
  updateAccelReadings();
}

void printData() {
  Serial.print(actualValueAngle);
  Serial.print(",");
  Serial.print(setValueAngle);
  Serial.print(",");
  Serial.println(pidAngleOutput);
  //delay(100);
}

void runMotors(double driveRate) {
  if (actualValueAngle > 0) {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, HIGH);
  } else {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
  }
  analogWrite(EN1, driveRate);
  analogWrite(EN2, driveRate);
}

void changeState(int newState) {
  currentState = newState;
}

void readEncoders() {
  unsigned long newPositionLeft = leftEncoder.read();
  unsigned long newPositionRight = rightEncoder.read();
  if (newPositionLeft != oldPositionLeft) {
    oldPositionLeft = newPositionLeft;
  }
  if (newPositionRight != oldPositionRight) {
    oldPositionRight = newPositionRight;
  }
}

void turnOffMotors() {
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
}

void turnOnMotors() {
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
}
