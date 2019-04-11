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
#include <Adafruit_10DOF.h>
#include <Wire.h>


// Assign a unique ID to the sensors
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

//See limitations of Arduino SoftwareSerial
#define HWSERIAL Serial1 // RX, TX (0, 1)

RoboClaw roboclaw(&HWSERIAL, 10000);
#define MOTOR_DRIVER 0x80
#define STOP_MOTOR 64

// Constants representing the states in the state machine
const int S_INIT = 0;
const int S_SETUP = 1;
const int S_RUNNING = 2;
int currentState = S_SETUP; // A variable holding the current state

// Variables storing sensor data
float roll;
uint32_t encoderCounts[2];

// Variables for recieving data
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars]; // An array to store the received data

// Variables holding buttonvalues
const byte numButtons = 5;
int buttonStates[numButtons] = {0, 0, 0, 0, 0};

// PID Controller Angle
#define PID_ANGLE_OUTPUT_LOW 0
#define PID_ANGLE_OUTPUT_HIGH 127
double kp_A = 4.0; double ki_A = 0.0; double kd_A = 0.0;
double actualValueAngle = 0.0; double setValueAngle = 2.25; double pidAngleOutput = 0.0;
PID pidAngle(&actualValueAngle, &pidAngleOutput, &setValueAngle, kp_A, ki_A, kd_A, REVERSE);

void setup() {
  Serial.begin(115200);
  roboclaw.begin(115200);
  initSensors();
  
  pidAngle.SetMode(AUTOMATIC);
  pidAngle.SetSampleTime(1);
  pidAngle.SetOutputLimits(PID_ANGLE_OUTPUT_LOW, PID_ANGLE_OUTPUT_HIGH);
}

void loop() {
  readStringFromSerial();
  updateButtonValues();
  
  roll = getAccurateRoll();

  switch (currentState) {

    case S_INIT:
      if (buttonStates[0] == 1) { // Middle button pressed
        changeState(S_SETUP);
      }
      break;

    case S_SETUP:
      // TODO: Still able to drive
      if ((buttonStates[5] == 1) || (abs(roll) < 45)) { // X button pressed
        changeState(S_RUNNING);
      }
      break;

    case S_RUNNING:
      // readEncoders();
      actualValueAngle = roll;
      pidAngle.Compute();
      driveMotor1(pidAngleOutput);
      driveMotor2(pidAngleOutput);

      if ((buttonStates[5] == 1) || (abs(roll) > 45)) { // X button pressed
        driveMotor1(STOP_MOTOR);
        driveMotor2(STOP_MOTOR);
        changeState(S_SETUP);
      }
      break;

    default:
      changeState(S_INIT);
  }

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
}

/**
  Calculate roll and roll from the raw accelerometer data
  @return roll in degrees
*/
float getAccurateRoll() {

  sensors_event_t accel_event;
  sensors_vec_t   orientation;
  float sum = 0;

  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    for (int i = 0; i <= 50; i++) {
      sum += orientation.roll;
    }
    return sum / 50;
  }
}

/**
  Reads the encoder position.
  @param encoder to read
  @return encoder position
*/
int32_t readEncoders() {
  uint8_t status1, status2, status3, status4;
  bool valid1, valid2, valid3, valid4;

  //Read all the data from Roboclaw before displaying on Serial Monitor window
  //This prevents the hardware serial interrupt from interfering with
  //reading data using software serial.
  int32_t enc1 = roboclaw.ReadEncM1(MOTOR_DRIVER, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(MOTOR_DRIVER, &status2, &valid2);
  encoderCounts[0] = enc1;
  encoderCounts[1] = enc2;
  // int32_t speed1 = roboclaw.ReadSpeedM1(MOTOR_DRIVER, &status3, &valid3);
  // int32_t speed2 = roboclaw.ReadSpeedM2(MOTOR_DRIVER, &status4, &valid4);
}
