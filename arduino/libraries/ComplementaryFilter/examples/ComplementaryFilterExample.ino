#include <ComplementaryFilter.h>

ComplementaryFilter complementaryFilter(1.5, 0.5, 0.005);

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
  
  complementaryFilter.in(measuredValue);
  float filterAngle = complementaryFilter.out();

  // use the Serial Ploter for a good visualization
  Serial.print(raw, 4);
  Serial.print(",");
  Serial.print(measuredValue, 4);
  Serial.print(",");
  Serial.print(filterAngle, 4);
  Serial.println();

}