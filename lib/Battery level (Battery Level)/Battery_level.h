// BATTERY_LEVEL.h
#ifndef BATTERY_LEVEL_H
#define BATTERY_LEVEL_H

#include <Arduino.h>

const int MAX_ANALOG_VAL = 4095;
const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2

class Battery_level {
  public:
    
    int battery_level_percent();
    double voltage_level();
    int raw_value();
    int rawValue;
    float voltageLevel;
    float batteryFraction;
    int batt_percentage;

  private:

};

#endif