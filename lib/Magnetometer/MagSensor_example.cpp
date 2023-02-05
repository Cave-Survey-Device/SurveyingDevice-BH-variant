#include <Arduino.h>
#include "RM3100.h"

RM3100 magSensor;

void setup() {
  Serial.begin(9600);
  magSensor.init();
  Serial.println("Finished setup");
}

void loop() {
  Vector3d vec = magSensor.get_mag_vec();
  magSensor.update();
  Serial.printf("X: %f   Y: %f   Z: %f\n", vec(0), vec(1), vec(2));
  delay(1000);
}