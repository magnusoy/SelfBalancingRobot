#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_10DOF.h>

// Assign a unique ID to the sensors
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

void setup() {
  // put your setup code here, to run once:
  initSensors();
  Serial.begin(38400);
}

void loop() {
  // put your main code here, to run repeatedly:
  double roll = getRoll();
  Serial.println(roll);


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

