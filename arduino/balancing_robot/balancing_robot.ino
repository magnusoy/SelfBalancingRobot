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
const int S_RUNNING = 1;
int currentState = S_INIT; // A variable holding the current state

double gyroY;
double accelY;
unsigned long t0 = micros();
unsigned long t1 = 0;
float filteredOutput;
const float ALPHA = 0.98;

// PID Controller Angle
#define PID_ANGLE_OUTPUT_LOW 0
#define PID_ANGLE_OUTPUT_HIGH 127
double kp = 65.0; double ki = 11.0; double kd = 0.0;
double actualValueAngle = 0.0; double setValueAngle = 0.0; double pidAngleOutput = 0.0;
PID pid(&actualValueAngle, &pidAngleOutput, &setValueAngle, kp, ki, kd, REVERSE);

void setup() {
  Serial.begin(115200);
  roboclaw.begin(115200);
  initSensors();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1);
  pid.SetOutputLimits(PID_ANGLE_OUTPUT_LOW, PID_ANGLE_OUTPUT_HIGH);
}

void loop() {
  updateSensors();
  actualValueAngle = complementaryFilter(gyroY, accelY, ALPHA);

  switch (currentState) {
    case S_INIT:
      if (abs(actualValueAngle) < 7) {
        changeState(S_RUNNING);
      }
      break;

    case S_RUNNING:
      pid.Compute();

      printShit(pidAngleOutput);

      driveMotor1(pidAngleOutput);
      driveMotor2(pidAngleOutput);

      if (abs(actualValueAngle) > 7) {
        driveMotor1(STOP_MOTOR);
        driveMotor2(STOP_MOTOR);
        changeState(S_INIT);
      }
      break;
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
  gyroY = event.gyro.x;

  accel.getEvent(&event);
  accelY = event.acceleration.y;
}

double complementaryFilter(double gyro, double accel, float alpha) {
  int dt = t1 - t0;
  filteredOutput = alpha * (filteredOutput + gyro * dt / 1000000) + (1 - alpha) * accel;
  t0 = t1;
  return filteredOutput;
}

void printShit(double newOutput) {
  Serial.print(actualValueAngle);
  Serial.print(",");
  Serial.print(setValueAngle);
  Serial.print(",");
  Serial.println(newOutput);
}
