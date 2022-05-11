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

#include "wifi_scanner.h"
#include "WiFi.h"
#include "esp_wifi.h"
wifi_scanner::wifi_scanner(ros::NodeHandle &nh)
  : nh_(nh)
{
}

void wifi_scanner::advertise()
{
  nh_.advertise(wifi_scan_pub);
}

void wifi_scanner::checkQueue()
{
  if (waiting > 0) {
    waiting--;
    wifi_scan_msg.data = msg_buf[waiting];
    wifi_scan_pub.publish(&wifi_scan_msg);
  }
}
/*
void wifi_scanner::checkZeroScan(int maximum)
{
  bool all_zero = true;
  for (int i = 0; i < n_channel; i++) {
    all_zero = all_zero && aps[i] == 0;
  }
  if (all_zero) {
    all_zero_count++;
    if (all_zero_count > maximum) {
      restart();
    }
  } else {
    all_zero_count = 0;
  }
}*/

void wifi_scanner::update() {
  if (isScanning == false) {
    /*if (!nh.connected()) {
      ESP.restart();
    }*/
    // TODO
    // not sure why, but when the serial is disconnected
    // sometimes it can be a strange state that nh.connected() == true and WiFi Scan does not work
    // restart the hardware if the WiFi scan returns no result for 10 consequtive cycles
    /*if (channel == 0) {
      checkZeroScan(10);
    }*/
    if (millis() <= scanningStart) {
      // during scan interval
      checkQueue();
      return;
    }

    if (count[channel] >= skip[channel]) {
      // start scan for the current channcel
      //
      // definition
      // int16_t scanNetworks(bool async = false,
      //                      bool show_hidden = false,
      //                      bool passive = false,
      //                      uint32_t max_ms_per_chan = 300,
      //                      uint8_t channel = 0);
      int n = WiFi.scanNetworks(true, false, false, scan_duration, channel + 1);
      scanningStart = millis();
      count[channel] = 0;
      isScanning = true;
      //showScanStatus();
    } else {
      // skip until skip count
      count[channel] += 1;
      channel = (channel + 1) % n_channel;
    }
  }
  else {
    int n = 0;
    if ((n = WiFi.scanComplete()) >= 0) {
      // scan completed
      aps[channel] = n;
      //showScanStatus();
      if (verbose) {
        sprintf(buf, "v[ch:%2d][%3dAPs][skip:%2d/%2d]%3dms,%5dms",
                channel + 1, n, skip[channel], max_skip,
                millis() - scanningStart, millis() - lastseen[channel]);
        nh_.loginfo(buf);
      }
      lastseen[channel] = millis();
      scanningStart = millis() + scan_interval;

      if (n == 0) {
        // increments skip count if no AP is found at the current channel
        skip[channel] = min(skip[channel] + 1, max_skip);
      } else {
        // if APs are found, put string into the queue
        skip[channel] = 0;
        for (int i = 0; i < n && waiting < MAX_WAITING; ++i) {
          sprintf(msg_buf[waiting], "%s,%s,%d,%d,%d,%d", WiFi.BSSIDstr(i).c_str(), WiFi.SSID(i).c_str(),
                  WiFi.channel(i), WiFi.RSSI(i), nh_.now().sec, nh_.now().nsec);
          waiting++;
        }
      }

      channel = (channel + 1) % n_channel;
      isScanning = false;
    } else {
      // waiting scan result
      checkQueue();
    }
  }
}
