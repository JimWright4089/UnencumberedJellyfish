
#ifndef __HARDWARE_COMM_HPP__
#define __HARDWARE_COMM_HPP__

#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "Jims_BNO055.h"
#include "utility/imumaths.h"

using namespace std::chrono_literals;
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class HardwareComm : public rclcpp::Node
{
public:
  HardwareComm();

private:
  void timerCallback();
  int mCount = 0; 
  rclcpp::TimerBase::SharedPtr mTimer;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr mPubIMUUncompensated;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr m_joy;
  Jims_BNO055 mBno = Jims_BNO055(-1, 0x28);
};

#endif