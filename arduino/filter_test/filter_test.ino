#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_10DOF.h>
#include <MedianFilter.h>


/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);

MedianFilter filter(31, 0);

void setup() {
  Serial.begin(115200);

  /* Initialise the sensors */
  initSensors();

}

void loop() {
  float pitch = getPitch();
  Serial.println(pitch);


}

void initSensors() {
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
}

float getPitch() {
  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation)) {
    filter.in(orientation.pitch);
    float pitch = filter.out();
    return pitch;
  }
}

