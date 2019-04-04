#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <MedianFilter.h>
#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000);

LSM9DS1 imu;

MedianFilter filterAy(61, 0);
MedianFilter filterAz(61, 0);

#define LSM9DS1_M  0x1E
#define LSM9DS1_AG  0x6B

#define address 0x80


// Defining global variables for recieving data
boolean newData = false;
const byte numChars = 16;
char receivedChars[numChars]; // An array to store the received data

int speedM1;
int speedM2;

void setup() {
  //Open roboclaw serial ports
  roboclaw.begin(38400);

  Serial.begin(115200);

  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  imu.begin();
}

void loop() {
  readStringFromSerial();
  if (newData) {
    speedM1 = getValueFromSerial(receivedChars, ',', 0).toInt();
    speedM2 = getValueFromSerial(receivedChars, ',', 1).toInt();
    newData = false;
  }

  if (imu.accelAvailable()) {
    imu.readAccel();
  }
  float roll = calculateRoll(imu.ay, imu.az);
  Serial.println(roll, 2);
}

/**
  docstring
*/
float calculateRoll(int ay_, int az_) {
  float ayFiltered = filterAy.in(ay_);
  float azFiltered = filterAz.in(az_);
  float roll = atan2(ayFiltered, azFiltered);
  roll  *= 180.0 / PI;
  return roll;
}

/**
  Drive motor 1 forward with the given speed.
  0 = stop
  64 = middle forward
  127 = full forward
  @param speed, 0 - 127
*/
void driveMotor1Forward(int speed_) {
  roboclaw.ForwardM1(address, speed_);
}

/**
  Drive motor 2 forward with the given speed.
  0 = stop
  64 = middle forward
  127 = full forward
  @param speed, 0 - 127
*/
void driveMotor2Forward(int speed_) {
  roboclaw.ForwardM2(address, speed_);
}

/**
  Drive motor 1 backward with the given speed.
  0 = stop
  64 = middle backward
  127 = full backward
  @param speed, 0 - 127
*/
void driveMotor1Backward(int speed_) {
  roboclaw.BackwardM1(address, speed_);
}

/**
  Drive motor 1 backward with the given speed.
  0 = stop
  64 = middle backward
  127 = full backward
  @param speed, 0 - 127
*/
void driveMotor2Backward(int speed_) {
  roboclaw.BackwardM2(address, speed_);
}

/**
  Drive motor 1 both directions with the given speed.
  0 = full backward
  64 = stop
  127 = full forward
  @param speed, 0 - 127
*/
void driveMotor1(int speed_) {
  roboclaw.ForwardBackwardM1(address, speed_);
}

/**
  Drive motor 2 both directions with the given speed.
  0 = full backward
  64 = stop
  127 = full forward
  @param speed, 0 - 127
*/
void driveMotor2(int speed_) {
  roboclaw.ForwardBackwardM2(address, speed_);
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
