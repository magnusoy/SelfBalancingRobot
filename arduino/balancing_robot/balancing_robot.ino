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
#include <SoftwareSerial.h>
#include <Wire.h>


// Assign a unique ID to the sensors
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10, 11); // RX, TX

RoboClaw roboclaw(&serial, 10000);
#define motorAddress 0x80

// Constants representing the states in the state machine
const int S_INIT = 0;
const int S_SETUP = 1;
const int S_RUNNING = 2;
int currentState = S_INIT; // A variable holding the current state

float roll;
uint32_t encoderCounts[2];

// Defining global variables for recieving data
boolean newData = false;
const byte numChars = 16;
char receivedChars[numChars]; // An array to store the received data

// Variables holding buttonvalues
const byte numButtons = 5;
int buttonStates[numButtons] = {0, 0, 0, 0, 0};

// PID Controller Encoder
double kp_E = 0.28; double ki_E = 0.1; double kd_E = 0.62;
double actualValueEncoder = 0.0; double setValueEncoder = 0.0; double PIDOutputEncoder = 0.0;
PID pidEncoder(&actualValueEncoder, &PIDOutputEncoder, &setValueEncoder, kp_E, ki_E, kd_E, REVERSE);

// PID Controller Angle
#define PID_ANGLE_OUTPUT_LOW 0
#define PID_ANGLE_OUTPUT_HIGH 127
double kp_A = 0.28; double ki_A = 0.1; double kd_A = 0.62;
double actualValueAngle = 0.0; double setValueAngle = 90.0; double pidAngleOutput = 0.0;
PID pidAngle(&actualValueAngle, &pidAngleOutput, &setValueAngle, kp_A, ki_A, kd_A, REVERSE);

void setup() {
  initSensors();
  roboclaw.begin(38400);
  Serial.begin(115200);

  pidEncoder.SetMode(MANUAL);
  pidAngle.SetMode(MANUAL);
  pidEncoder.SetSampleTime(100);
  pidAngle.SetSampleTime(10);
  pidAngle.SetOutputLimits(PID_ANGLE_OUTPUT_LOW, PID_ANGLE_OUTPUT_HIGH);
}

void loop() {
  readStringFromSerial();
  updateButtonValues(buttonStates);
  roll = getRoll();

  switch (currentState) {

    case S_INIT:
      if (buttonStates[0] == 1) { // Middle button pressed
        changeState(S_SETUP);
      }
      break;

    case S_SETUP:
      // TODO: Still able to drive
      if ((buttonStates[5] == 1) && (abs(roll) < 45)) { // X button pressed
        // TODO: Be able to rise up
        pidEncoder.SetMode(AUTOMATIC);
        pidAngle.SetMode(AUTOMATIC);
        changeState(S_RUNNING);
      }
      break;

    case S_RUNNING:
      readEncoders();
      pidAngle.Compute();
      driveMotor1(pidAngleOutput);
      driveMotor2(pidAngleOutput);
      if (buttonStates[1] == 1) { // Forward button pressed
        // TODO: Drive forward
      } else if (buttonStates[2] == 1) { // Backward button pressed
        // TODO: Drive backward
      } else if (buttonStates[3] == 1) { // Left button pressed
        // TODO: Drive left
      } else if (buttonStates[4] == 1) { // Right button pressed
        // TODO: Drive right
      }
      if ((buttonStates[5] == 1) || (abs(roll) > 45)) { // X button pressed
        pidEncoder.SetMode(MANUAL);
        pidAngle.SetMode(MANUAL);
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
  Fetches the value from a substring,
  wich is seperated with a given symbol.
  @param data your String to be seperated
  @param seperator your symbol to seperate by
  @param index where your value is located
  @return substring before seperator
*/
String getValueFromSerial(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  strIndex[0] = strIndex[1] + 1;
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/**
  Drive motor 1 both directions with the given speed.
  0 = full backward
  64 = stop
  127 = full forward
  @param speed, 0 - 127
*/
void driveMotor1(int speed_) {
  roboclaw.ForwardBackwardM1(motorAddress, speed_);
}

/**
  Drive motor 2 both directions with the given speed.
  0 = full backward
  64 = stop
  127 = full forward
  @param speed, 0 - 127
*/
void driveMotor2(int speed_) {
  roboclaw.ForwardBackwardM2(motorAddress, speed_);
}

void changeState(int newState) {
  currentState = newState;
}

/**
  Reads from the recived char array and
  fills an array with the stored values.
*/
void updateButtonValues(int values[]) {
  if (newData) {
    for (int index = 0; index < 5; index++) {
      values[index] = getValueFromSerial(receivedChars, ',', index).toInt();
    }
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
float getRoll() {

  sensors_event_t accel_event;
  sensors_vec_t   orientation;
  float sum = 0;

  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    for (int i = 0; i <= 10; i++) {
      sum += orientation.roll;
    }
    return sum / 10;
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
  int32_t enc1 = roboclaw.ReadEncM1(motorAddress, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(motorAddress, &status2, &valid2);
  encoderCounts[0] = enc1;
  encoderCounts[1] = enc2;
  // int32_t speed1 = roboclaw.ReadSpeedM1(motorAddress, &status3, &valid3);
  // int32_t speed2 = roboclaw.ReadSpeedM2(motorAddress, &status4, &valid4);
}
