// Copyright (c) 2020  Carnegie Mellon University, IBM Corporation, and others
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
#ifndef wifi_scanner_h
#define wifi_scenner_h

#ifdef ESP32
#undef ESP32
#include <ros.h>
#define ESP32
#else
#include <ros.h>
#endif
#include "Arduino.h"

#include "std_msgs/String.h"

// baud rate for serial connection
#define BAUDRATE (115200)

// maximum number of channels
#define MAX_CHANNEL (13)

// 11 ch in US, 13 ch in other countries
#define DEFAULT_N_CHANNEL (11)

// typical WiFi SSID beacon is 102.4 ms
#define DEFAULT_SCAN_DURATION (105)

// need some interval if you want to transport via WiFi
#define DEFAULT_SCAN_INTERVAL (5)

// maximum wait cycle
// if you have APs in 3 channels in your environment, the maximum wait time is like
// (3*DEFAULT_MAX_SKIP)*(DEFAULT_SCAN_DURATION+SCAN_INTERVAL) = (3*14)*105 = 42*105 =
#define DEFAULT_MAX_SKIP (14)

// maximum queue size, ignore if exceeds
#define MAX_WAITING (128)

// verbosity
#define DEFAULT_VERBOSITY (true)

// declare reset function @ address 0
// this can cause undesirable result with multiple core system like ESP32
// void(* resetFunc) (void) = 0;

#define PARAM_TIMEOUT (100)
class wifi_scanner {
  private:
    ros::NodeHandle &nh_;
    std_msgs::String wifi_scan_msg;
    ros::Publisher wifi_scan_pub = ros::Publisher("wifi_scan_str", &wifi_scan_msg);
    int max_skip = DEFAULT_MAX_SKIP;
    int n_channel = DEFAULT_N_CHANNEL;
    int scan_duration = DEFAULT_SCAN_DURATION;
    int scan_interval = DEFAULT_SCAN_INTERVAL;
    bool verbose = DEFAULT_VERBOSITY;

    bool isScanning = false;
    unsigned long scanningStart = 0;
    int channel = 0;
    int skip[MAX_CHANNEL];
    int count[MAX_CHANNEL];
    int aps[MAX_CHANNEL];
    unsigned long lastseen[MAX_CHANNEL];
    char buf[256];

    // BSSID=17, SSID=32, CH=2, RSSI=4, sec=10, nsec=10, commas=5, total 80 + margin 20
    char msg_buf[MAX_WAITING][100];
    int waiting = 0;
    int all_zero_count = 0;

    void checkQueue();
    //void checkZeroScan(int maximum);
  public:
    wifi_scanner(ros::NodeHandle &nh);
    void advertise();
    void update();

};

#endif//wifi_scanner_h