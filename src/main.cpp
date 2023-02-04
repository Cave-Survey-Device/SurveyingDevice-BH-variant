// main code: ArduinoCode.ino
#include "OLED.h"
#include "BNO085.h"
#include "lidar.h"
#include "Battery_level.h"
#include "esp_sleep.h"

#define TAKE_READING_BUTTON 32
#define LASER_ON_OFF_BUTTON 27

OLED oled; // create a OLED object
BNO085 bno085;
Lidar lidar;
Battery_level battery_level;

volatile bool interrupt_button_pressed = 0;
volatile bool interrupt_button_pressed_timer = 0;
volatile bool laser_get_measurment = 0;

unsigned long timer_start = 0;
unsigned long current_time = 0;
unsigned long interval = 3000;

float new_combined_value;
float combined_value;
float threshold = 0.1;
float diff;

float distance = 0;
float compass;
float clino;
int sensor_status;
bool ble_status;
int batt_percentage;

void Laser_on_off_interupt()
{
  lidar.toggle_laser();
}

void take_reading_button_pressed()
{
  if (digitalRead(TAKE_READING_BUTTON) == 0)
  {
    interrupt_button_pressed = 1;
    interrupt_button_pressed_timer = 1;
  }
  else
  {
    interrupt_button_pressed = 0;
  }
}

void setup()
{
  Serial.begin(9600);
  oled.Initialise();
  bno085.Initialise();
  oled.Distance(distance);
  lidar.init();
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27,1); //1 = High, 0 = Low


  pinMode(GPIO_NUM_14, OUTPUT);
  digitalWrite(GPIO_NUM_14, HIGH);

  pinMode(TAKE_READING_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TAKE_READING_BUTTON), take_reading_button_pressed, CHANGE);

  pinMode(LASER_ON_OFF_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LASER_ON_OFF_BUTTON), Laser_on_off_interupt, FALLING);
}

void loop()
{

  // this is if the user holds down the button
  while (interrupt_button_pressed == 1)
  {
    if (interrupt_button_pressed_timer == 1)
    {
      timer_start = millis();
      interrupt_button_pressed_timer = 0;
      laser_get_measurment = 1;
    }
    current_time = millis();
    Serial.println(current_time - timer_start);

    if (current_time - timer_start > interval)
    {
      oled.clearDisplay();
      lidar.laser_turn_off();
      digitalWrite(GPIO_NUM_14, LOW);
      esp_deep_sleep_start();
      laser_get_measurment = 0;
      break;
    }
  }

  current_time = millis();
  timer_start = millis();

  if (laser_get_measurment == 1)
  {
    distance = lidar.get_measurement();
    oled.Distance(distance);
    laser_get_measurment = 0;
    delay(100);
    lidar.toggle_laser();

  }

  ble_status = random(0, 100);
  batt_percentage = battery_level.battery_level_percent();

  compass = bno085.Compass();
  clino = bno085.Clino();

  combined_value = compass + clino;
  diff = combined_value - new_combined_value;

  if (diff > threshold || diff < -threshold)
  {

    oled.Compass(compass);
    oled.Clino(-clino);
    compass = bno085.Compass();
    clino = bno085.Clino();
    new_combined_value = compass + clino;
    oled.Blutooth(ble_status);
    oled.Battery(batt_percentage);
    sensor_status = bno085.sensor_cal_status();
    oled.Sensor_cal_status(sensor_status);
  }

}