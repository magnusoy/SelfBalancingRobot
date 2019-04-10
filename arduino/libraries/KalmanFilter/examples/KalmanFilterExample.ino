#include <KalmanFilter.h>

KalmanFilter kalmanFilter(2, 2, 0.01);

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 100;
long refresh_time;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // read a reference value from A0 and map it from 0 to 100
  float raw = analogRead(A0) / 1024.0 * 100.0;
  
  // add a noise to the reference value and use as the measured value
  float measuredValue = raw + random(-100, 100) / 100.0;

  // calculate the estimated value with Kalman Filter
  
  kalmanFilter.in(measuredValue);
  float estimatedValue = kalmanFilter.out();

  // send to Serial output every 100ms
  // use the Serial Ploter for a good visualization
  if (millis() > refresh_time) {
    Serial.print(raw, 4);
    Serial.print(",");
    Serial.print(measuredValue, 4);
    Serial.print(",");
    Serial.print(estimatedValue, 4);
    Serial.println();
    
    refresh_time = millis() + SERIAL_REFRESH_TIME;
  }

}