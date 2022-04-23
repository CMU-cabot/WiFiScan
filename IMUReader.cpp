/*******************************************************************************
 * Copyright (c) 2020  Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include "IMUReader.h"

#define D2R 0.0174532925

IMUReader::IMUReader(ros::NodeHandle &nh):
  SensorReader(nh),
  imu_pub_("imu_raw", &imu_msg_),
  calibration_pub_("calibration", &calibration_msg_)
{
  nh_.advertise(imu_pub_);
  nh_.advertise(calibration_pub_);
}

void IMUReader::calibration() {
  in_calibration_ = true;
  init();
}

void IMUReader::init() {
  init(NULL);
}

void IMUReader::init(uint8_t *offsets) {
  if(!imu_.begin())
  {
    nh_.loginfo("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return;
  }
  initialized_ = true;
  if (offsets != NULL) {
    imu_.setSensorOffsets(offsets);
  }
  imu_.setExtCrystalUse(true);

  // time 2 + orientation 4 + angular_velocy 3 + linear_acceleration 3
  imu_msg_.data = (float*)malloc(sizeof(float)*12);
  imu_msg_.data_length = 12;

  calibration_msg_.data = (uint8_t*)malloc(sizeof(uint8_t)*26);
  calibration_msg_.data_length = 26;
}

void IMUReader::update() {
  if (!initialized_) {
    return;
  }
  // put int32 as float32
  auto timestamp = nh_.now();
  imu_msg_.data[0] = *((float*)(&timestamp.sec));
  imu_msg_.data[1] = *((float*)(&timestamp.nsec));
  
  imu::Quaternion q = imu_.getQuat();

  imu_msg_.data[2] = q.x();
  imu_msg_.data[3] = q.y();
  imu_msg_.data[4] = q.z();
  imu_msg_.data[5] = q.w();

  imu::Vector<3> xyz = imu_.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  imu_msg_.data[6] = xyz.x()*D2R;
  imu_msg_.data[7] = xyz.y()*D2R;
  imu_msg_.data[8] = xyz.z()*D2R;
    
  xyz = imu_.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  imu_msg_.data[9] = xyz.x();
  imu_msg_.data[10] = xyz.y();
  imu_msg_.data[11] = xyz.z();

  // publish
  imu_pub_.publish( &imu_msg_ );
}

void IMUReader::update_calibration() {
  if (!initialized_) {
    return;
  }

  uint8_t *offsets = calibration_msg_.data;
  imu_.getSensorOffsets(offsets);
  imu_.getCalibration(offsets+22, offsets+23, offsets+24, offsets+25);

  calibration_pub_.publish( &calibration_msg_);
}
