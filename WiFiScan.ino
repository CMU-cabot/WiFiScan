// Copyright (c) 2020  Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

//
// This Arduino code will publish WiFi scan result nas string message
// topic: wifi_scan_str
// format: BSSID,SSID,Channel,RSSI,sec,nsec
//

// https://github.com/ros-drivers/rosserial/pull/448
// #define ESP_SERIAL
// still not included in released version 0.9.1
// the following is a workaround
#undef ESP32
#include "ros.h"
#define ESP32

#include "Arduino.h"
#include "IMUReader.h"
#include "WiFiReader.h"
#include <arduino-timer.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET     4
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// baud rate for serial connection
#define BAUDRATE (115200)

// declare reset function @ address 0
// this can cause undesirable result with multiple core system like ESP32
// void(* resetFunc) (void) = 0;

bool is_display_available = false;

ros::NodeHandle nh;
IMUReader imuReader(nh);
WiFiReader wifiReader(nh);
Timer<10> timer;
#define IMU_COUNT_NUM 500
double imu_count[IMU_COUNT_NUM];
int imu_index = 0;
double imu_freq = 0;

void loginfo(char *buf)
{
  nh.loginfo(buf);

  if (!is_display_available) {
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F(buf));
  display.display();
}

void showText(const char *buf, int row)
{
  if (!is_display_available) {
    return;
  }
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 8*row);
  display.println(F(buf));
}

void showAppStatus()
{
  char buf[128];
  display.clearDisplay();
  showText("WiFi Scanner Ready", 0);
  showText("Waiting Connection", 1);
  sprintf(buf, "Time: %7.1f", millis()/1000.0);
  showText(buf, 2);
  display.display();
}

void setup()
{
  memset(imu_count, 0, sizeof(imu_count));

  if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    is_display_available = true;
    showAppStatus();
  }
    
  // init hardware
  nh.getHardware()->setBaud(BAUDRATE);
  // init rosserial
  nh.initNode();
  nh.setSpinTimeout(100);

  int wait = millis();
  while(!nh.connected()) {
    showAppStatus();
    nh.spinOnce();
    delay(100);

    // TODO
    // very rarely, it can be astate that the serial cannot be connected
    // so reset every 10 seconds
    if (millis() - wait > 10*1000) {
      restart();
    }
  }

  int run_imu_calibration = 0;
  nh.getParam("~run_imu_calibration", &run_imu_calibration, 1, 500);
  if (run_imu_calibration != 0) {
    imuReader.calibration();
    timer.every(100, [](void*){
      imuReader.update();
      imuReader.update_calibration();
      return true;
    });
    nh.loginfo("Calibration Mode started");
    return;
  }

  int calibration_params[22];
  uint8_t *offsets = NULL;
  if (nh.getParam("~calibration_params", calibration_params, 22, 500)) {
    offsets = (uint8_t*)malloc(sizeof(uint8_t) * 22);
    for(int i = 0; i < 22; i++) {
      offsets[i] = calibration_params[i] & 0xFF;
    }
  } else {
    nh.logwarn("clibration_params is needed to use IMU (BNO055) correctly.");
    nh.logwarn("You can run calibration by setting _run_imu_calibration:=1");
    nh.logwarn("You can check calibration value with /calibration topic.");
    nh.logwarn("First 22 byte is calibration data, following 4 byte is calibration status for");
    nh.logwarn("System, Gyro, Accel, Magnet, 0 (not configured) <-> 3 (configured)");
    nh.logwarn("Specify like calibration_params:=[0, 0, 0, 0 ...]");
    nh.logwarn("Visit the following link to check how to calibrate sensoe");
    nh.logwarn("https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/device-calibration");
  }
  nh.loginfo("setting up WiFi");
  wifiReader.init([](char *buf){
      if (!is_display_available) {
        return;
      }
      display.clearDisplay();
      sprintf(buf+strlen(buf), " IMU:%3.0fHz", imu_freq);
      showText(buf, 0);
      display.display();
    });
  
  nh.loginfo("setting up BNO055");
  imuReader.init(offsets);

  delay(100);

  timer.every(9, [](void*){
      imuReader.update();
      imu_freq = IMU_COUNT_NUM / (nh.now().toSec() - imu_count[imu_index]);
      imu_count[imu_index] = nh.now().toSec();
      imu_index = (imu_index+1)%IMU_COUNT_NUM;
      return true;
    });

  // is_display_available could be changed from true to false somehow while init
  // so, this is a workaround to re-enable display
  if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    is_display_available = true;
  }
}

void loop()
{
  timer.tick<void>();
  wifiReader.update();
  nh.spinOnce();
}

void restart()
{
  display.clearDisplay();
  showText("Restart ESP", 0);
  display.display();
  ESP.restart();
}
