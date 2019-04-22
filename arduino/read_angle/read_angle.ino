#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

// Assign a unique ID to the sensors
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

double gyroY;
double accelY;
unsigned long t0 = micros();
unsigned long t1 = 0;
float filteredOutput;

void setup() {
  initSensors();
  Serial.begin(9600);
}

void loop() {
  updateSensors();
  double angle = complementaryFilter(gyroY, accelY);
  Serial.print(accelY);
  Serial.print(",");
  Serial.println(angle);
  
  delay(100);

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


double complementaryFilter(double gyro, double accel) {
  int dt = t1 - t0;
  filteredOutput = 0.9 * (filteredOutput + gyro * dt / 1000000) + 0.1 * accel;
  t0 = t1;
  return filteredOutput;
}
