#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <KalmanFilter.h>
#include <PID_v1.h>
#include <Filters.h>
#include <Encoder.h>
#include <digitalWriteFast.h>

// Defining motor and encoder pins
#define EN1 10
#define EN2 9
#define DIR1 12
#define DIR2 7

#define S1A  2
#define S2A  5
#define S1B  3
#define S2B  6

Encoder rightEncoder(S1A, S1B);
Encoder leftEncoder(S2A, S2B);

long oldPositionRight = -999;
long oldPositionLeft = -999;


long speedLeft = 0;
long speedRight = 0;
// filters out changes faster that 1.6 Hz.
float filterFrequency = 1.6;

// create a one pole (RC) lowpass filter
FilterOnePole lowpassFilter(LOWPASS, filterFrequency);

// Use the LSM9DS1 class to create an object.
LSM9DS1 imu;

// Use the KalmanFilter class to create an object.
KalmanFilter kalmanFilter(2, 2, 0.1);
KalmanFilter motorFilter(2, 2, 0.1);

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B // Would be 0x6A if SDO_AG is LOW


// PID Controller Angle
#define PID_ANGLE_OUTPUT_LOW -255.0
#define PID_ANGLE_OUTPUT_HIGH 255.0
double kp = 8.0; double ki = 0.0; double kd = 0.1;
double actualValueAngle = 0.0; double setValueAngle = 104.3; double pidAngleOutput = 0.0;
PID pid(&actualValueAngle, &pidAngleOutput, &setValueAngle, kp, ki, kd, REVERSE);

// PID Controller Speed
double kp_S = 50; double ki_S = 0.0; double kd_S = 0.0;
double actualValueSpeed = 0.0; double setValueSpeed = 0.0; double PIDSpeedOutput = 0.0;
PID pidSpeed(&actualValueSpeed, &PIDSpeedOutput, &setValueSpeed, kp_S, ki_S, kd_S, REVERSE);

long t0 = 0;

// Constants representing the states in the state machine
const int S_INIT = 0;
const int S_RUNNING = 1;
int currentState = S_INIT; // A variable holding the current state

void setup() {
  Serial.begin(19200);
  initSensors();
  initMotorShield();

  pid.SetMode(AUTOMATIC);
  pidSpeed.SetMode(AUTOMATIC);
  pid.SetSampleTime(5);
  pidSpeed.SetSampleTime(5);
  pid.SetOutputLimits(PID_ANGLE_OUTPUT_LOW, PID_ANGLE_OUTPUT_HIGH);
  pidSpeed.SetOutputLimits(103.3, 105.3);
}

void loop() {
  switch (currentState) {
    case S_INIT:
      updateSensors();
      filterAngle();
      turnOffMotors();
      if ((abs(actualValueAngle) < 160) || (abs(actualValueAngle) > 60)) {
        changeState(S_RUNNING);
      }
      break;
    case S_RUNNING:
      pidSpeed.Compute();
      setValueAngle = PIDSpeedOutput;
      updateSensors();
      filterAngle();
      pid.Compute();
      motorFilter.in(pidAngleOutput);
      double drive = motorFilter.out();
      runMotors(drive);
      actualValueSpeed = abs(drive);
      if ((abs(actualValueAngle) > 160) || (abs(actualValueAngle) < 60)) {
        turnOffMotors();
        resetPosition();
        changeState(S_INIT);
      }

      break;
  }
  //printData();
  //Serial.println(actualValueAngle);
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
  pinModeFast(EN1, OUTPUT);
  pinModeFast(EN2, OUTPUT);
  pinModeFast(DIR1, OUTPUT);
  pinModeFast(DIR2, OUTPUT);

  digitalWriteFast(EN1, LOW);
  digitalWriteFast(EN2, LOW);
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
}

void runMotors(double driveRate) {
  double driveOutput = driveRate;
  if (actualValueAngle > setValueAngle) {
    digitalWriteFast(DIR1, LOW);
    digitalWriteFast(DIR2, LOW);
  } else {
    digitalWriteFast(DIR1, HIGH);
    digitalWriteFast(DIR2, HIGH);
    driveOutput *= -1;
  }
  analogWrite(EN1, driveOutput);
  analogWrite(EN2, driveOutput);
}

void changeState(int newState) {
  currentState = newState;
}

void readEncoders() {
  long newPositionLeft = leftEncoder.read();
  long newPositionRight = rightEncoder.read();
  readSpeed(newPositionLeft, newPositionRight);
  if (newPositionLeft != oldPositionLeft) {
    oldPositionLeft = newPositionLeft;
  }
  if (newPositionRight != oldPositionRight) {
    oldPositionRight = newPositionRight;
  }

}

void turnOffMotors() {
  digitalWriteFast(EN1, LOW);
  digitalWriteFast(EN2, LOW);
}

void turnOnMotors() {
  digitalWriteFast(EN1, HIGH);
  digitalWriteFast(EN2, HIGH);
}

void filterAngle() {
  double angle = getAngle(imu.ay, imu.az);
  kalmanFilter.in(angle);
  double kalmanAngle = kalmanFilter.out();
  lowpassFilter.input(kalmanAngle);
  actualValueAngle = lowpassFilter.output();
  actualValueAngle *= -1;
}

void readSpeed(long newPositionLeft, long newPositionRight) {
  long t1 = millis();
  speedLeft = (newPositionLeft - oldPositionLeft) * 1000 / (t1 - t0);
  speedRight = (newPositionRight - oldPositionRight) * 1000 / (t1 - t0);
  t0 = t1;
}

void resetPosition() {
  leftEncoder.write(0);
  rightEncoder.write(0);
}
