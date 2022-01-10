//----------------------------------------------------------------------------
//
//  $Workfile: drive_base.cpp$
//
//  $Revision: X$
//
//  Project:    Unencumbered Jellyfish
//
//                            Copyright (c) 2022
//                               James A Wright
//                            All Rights Reserved
//
//  Modification History:
//  $Log:
//  $
//
//  Notes:
//     This is the ROS2 Node for the Unencumbered Jellyfish's drive base
//
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  Includes
//----------------------------------------------------------------------------
#include "drive_date.hpp"


class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("drive_base"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("sinValue", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  double mCount = 0.0; 
  void timer_callback()
  {
    double sinValue = sin(mCount);
    mCount += (3.1415927 / 200); 
    auto message = std_msgs::msg::Float64();
    message.data = sinValue;
    RCLCPP_INFO(this->get_logger(), "%7.4f", message.data);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  size_t count_;
};

//--------------------------------------------------------------------
// Purpose:
//     Main entry point
//
// Notes:
//     None.
//--------------------------------------------------------------------
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
