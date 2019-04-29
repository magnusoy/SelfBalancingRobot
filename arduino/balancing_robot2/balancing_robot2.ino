/**
  MIT License

  Copyright (c) 2019 magnusoy

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#include "RoboClaw.h"
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <KalmanFilter.h>


// Assign a unique ID to the sensors
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

//#define HWSERIAL Serial1 // RX, TX (0, 1)
//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10, 11);

RoboClaw roboclaw(&serial, 10000);
#define MOTOR_DRIVER 0x80
#define STOP_MOTOR 64

KalmanFilter kalmanFilter(2, 2, 0.01);

// Constants representing the states in the state machine
const int S_INIT = 0;
const int S_SETUP = 1;
const int S_RUNNING = 2;
int currentState = S_SETUP; // A variable holding the current state

// Variables storing sensor data
uint32_t encoderCounts[2];
uint32_t motorSpeeds[2];

double gyroY;
double accelY;
unsigned long t0 = micros();
unsigned long t1 = 0;
float filteredOutput;
const float ALPHA = 0.95;

// Variables for recieving data
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars]; // An array to store the received data

// Variables holding buttonvalues
const byte numButtons = 5;
int buttonStates[numButtons] = {0, 0, 0, 0, 0};

// PID Controller Speed
double kp_S = 900; double ki_S = 0.0; double kd_S = 0.0;
double actualValueSpeed = 0.0; double setValueSpeed = 0.0; double PIDSpeedOutput = 0.0;
PID pidSpeed(&actualValueSpeed, &PIDSpeedOutput, &setValueSpeed, kp_S, ki_S, kd_S, DIRECT);

// PID Controller Angle
#define PID_OUTPUT_LOW 0
#define PID_OUTPUT_HIGH 127
double kp_A = 90.0; double ki_A = 0.01; double kd_A = 0.08;
double actualValueAngle = 0.0; double setValueAngle = 0.03; double pidAngleOutput = 0.0;
PID pidAngle(&actualValueAngle, &pidAngleOutput, &setValueAngle, kp_A, ki_A, kd_A, REVERSE);

void setup() {
  roboclaw.begin(115200);
  Serial.begin(9600);
  initSensors();

  pidAngle.SetMode(AUTOMATIC);
  pidSpeed.SetMode(AUTOMATIC);


  pidAngle.SetSampleTime(4);
  pidSpeed.SetSampleTime(4);
  pidAngle.SetOutputLimits(PID_OUTPUT_LOW, PID_OUTPUT_HIGH);
  pidSpeed.SetOutputLimits(-2, 2);
}

void loop() {
  readStringFromSerial();
  updateButtonValues();
  updateSensors();
  actualValueAngle = complementaryFilter(gyroY, accelY, ALPHA);

  switch (currentState) {

    case S_INIT:
      if (buttonStates[0] == 1) { // Middle button pressed
        changeState(S_SETUP);
      }
      break;

    case S_SETUP:
      if ((buttonStates[5] == 1) || (abs(actualValueAngle) <= 7)) { // X button pressed
        changeState(S_RUNNING);
      }
      break;

    case S_RUNNING:

      actualValueSpeed = motorSpeeds[0];
      pidSpeed.Compute();

      setValueAngle = PIDSpeedOutput;
      pidAngle.Compute();

      readEncoders();
      //kalmanFilter.in(pidAngleOutput);
      //double newOutput = kalmanFilter.out();

      driveMotor1(pidAngleOutput);
      driveMotor2(pidAngleOutput);

      if (buttonStates[1] == 1) { // Forward button pressed
        
      } else if (buttonStates[2] == 1) { // Backward button pressed
        
      } else if (buttonStates[3] == 1) { // Left button pressed
        // TODO: Drive left
      } else if (buttonStates[4] == 1) { // Right button pressed
        // TODO: Drive right
      }

      if ((buttonStates[5] == 1) || (abs(actualValueAngle) >= 7)) { // X button pressed
        driveMotor1(STOP_MOTOR);
        driveMotor2(STOP_MOTOR);
        changeState(S_SETUP);
      }
      break;
  }
  //Serial.println(actualValueAngle);
}

/**
  Reads a string from Serial Monitor.
*/
void readStringFromSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while ((Serial.available() > 0) && (!newData)) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // Terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

/**
  Drive motor 1 both directions with the given speed.
  0 = full backward
  64 = stop
  127 = full forward
  @param speed, 0 - 127
*/
void driveMotor1(int speed_) {
  roboclaw.ForwardBackwardM1(MOTOR_DRIVER, speed_);
}

/**
  Drive motor 2 both directions with the given speed.
  0 = full backward
  64 = stop
  127 = full forward
  @param speed, 0 - 127
*/
void driveMotor2(int speed_) {
  roboclaw.ForwardBackwardM2(MOTOR_DRIVER, speed_);
}

void changeState(int newState) {
  currentState = newState;
}

/**
  Reads from the recived char array and
  fills an array with the stored values.
*/
void updateButtonValues() {
  if (newData) {
    buttonStates[0] = receivedChars[0];
    buttonStates[1] = receivedChars[2];
    buttonStates[2] = receivedChars[4];
    buttonStates[3] = receivedChars[6];
    buttonStates[4] = receivedChars[8];
  }
  newData = false;
}

/**
  Initialize sensors.
*/
void initSensors() {
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }

  if (!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.print("Ooops, no L3GD20 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

void updateSensors() {
  sensors_event_t event;
  gyro.getEvent(&event);
  gyroY = event.gyro.y;

  accel.getEvent(&event);
  accelY = event.acceleration.y;
}
double complementaryFilter(double gyro, double accel, float alpha) {
  int dt = t1 - t0;
  filteredOutput = alpha * (filteredOutput + gyro * dt / 1000000) + (1 - alpha) * accel;
  t0 = t1;
  return filteredOutput;
}

/**
  Reads the encoder position.
  @param encoder to read
  @return encoder position
*/
void readEncoders() {
  uint8_t status1, status2, status3, status4;
  bool valid1, valid2, valid3, valid4;

  //Read all the data from Roboclaw before displaying on Serial Monitor window
  //This prevents the hardware serial interrupt from interfering with
  //reading data using software serial.
  int32_t enc1 = roboclaw.ReadEncM1(MOTOR_DRIVER, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(MOTOR_DRIVER, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(MOTOR_DRIVER, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(MOTOR_DRIVER, &status4, &valid4);
  encoderCounts[0] = enc1;
  encoderCounts[1] = enc2;
  motorSpeeds[0] = speed1;
  motorSpeeds[1] = speed2;

}
