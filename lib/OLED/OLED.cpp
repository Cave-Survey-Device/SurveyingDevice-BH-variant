// OLED.cpp
#include "OLED.h"



OLED::OLED() {
}

void OLED::Initialise() {
  display.begin(0x3D, true); // Address 0x3D default
  display.setTextSize(2); // 2:1 pixel scale
  display.setTextColor(SSD1327_WHITE); // Tells display to draw white text
  display.setTextColor(SSD1327_WHITE, SSD1327_BLACK); //sets the text background colour to black to overwrite existing numbers, prevents flicker. 
  //display.setContrast (0); // dim display (optional)
  display.clearDisplay();
  display.display();
}

void OLED::Distance(double distance) {
  display.fillRect(70, 30, 58, 98, SSD1327_BLACK); //clears the old distance value
  display.setTextSize(3);
  display.setCursor(0, 34);
  display.print(distance);
  display.println("m");
  display.display();
}

void OLED::Compass(double compass) {
  display.fillRect(70, 70, 58, 58, SSD1327_BLACK); //clears the old compass value
  display.setTextSize(3);
  display.setCursor(0, 70);
  display.print(compass);
  display.print((char)247); // degree symbol 
  display.display();
}

void OLED::Clino(double clino) {
  display.fillRect(70, 106, 58, 22, SSD1327_BLACK); //clears the old clino value
  display.setTextSize(3);
  display.setCursor(0, 106);
  display.print(clino);
  display.print((char)247); // degree symbol 
  display.display();
}

void OLED::Sensor_cal_status(int sensor_status) {
  display.setTextSize(2);
  display.setCursor(23, 4);
  display.print(sensor_status);
  display.display();
}

void OLED::Blutooth(bool ble_status) {
  //insert switch case to update
const unsigned char PROGMEM Bluetooth_icon [] = {
  0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x01, 0xfe, 0x00, 0x07, 0xff, 0x00, 0x07, 0xcf, 0x80, 0x0f, 
  0xc7, 0xc0, 0x0f, 0xc3, 0xc0, 0x0e, 0xd9, 0xc0, 0x0f, 0x03, 0xc0, 0x0f, 0x87, 0xc0, 0x0f, 0x87, 
  0xc0, 0x0f, 0xcf, 0xc0, 0x0f, 0x87, 0xc0, 0x0f, 0x03, 0xc0, 0x0e, 0x59, 0xc0, 0x0f, 0xc3, 0xc0, 
  0x0f, 0xc7, 0xc0, 0x07, 0xcf, 0x80, 0x07, 0xff, 0x80, 0x03, 0xff, 0x00, 0x00, 0xfe, 0x00, 0x00, 
  0x00, 0x00
};

//display.fillRect(0, 0, 22, 22, SSD1327_BLACK); //clears the area around the blutooth symbol
//display.display();
//delay(100);
display.drawBitmap(0,0,Bluetooth_icon, 22, 22, 10);
display.display();
}

void OLED::Battery(int batt_percentage) {
  display.drawRect(90, 0, 32, 15, SSD1327_WHITE);
  display.drawRect(122, 4, 3, 6, SSD1327_WHITE);
  batt_level = (batt_percentage/100.00)*32;
  display.fillRect(90, 0, batt_level, 15, SSD1327_WHITE);
  display.fillRect(91+batt_level, 1, 30-batt_level, 13, SSD1327_BLACK);
  display.setTextSize(2);
  display.setCursor(50, 0);
  display.print(batt_percentage);
  display.print("%");
  display.display();
}

void OLED::clearDisplay() {
   display.clearDisplay();
}