// BNO085.h
#ifndef BNO085_H
#define BNO085_H

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

void setReports(sh2_SensorId_t reportType, long report_interval);

class BNO085 {
  public:

    BNO085();
    void Initialise();
    float Compass();
    float Clino();
    int sensor_cal_status();

    float compass;
    float clino;
    int sensor_status;

    float old_val;
    float new_val;
    

    struct euler_t {
    float yaw;
    float pitch;
    float roll;
  } ypr;

  private:

  void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees);
  void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees);
  void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees);
  
};

#endif