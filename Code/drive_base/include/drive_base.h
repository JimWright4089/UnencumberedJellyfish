//----------------------------------------------------------------------------
//
//  $Workfile: drive_base.hpp$
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
#ifndef DRIVE_BASE_HPP
#define DRIVE_BASE_HPP

//----------------------------------------------------------------------------
//  Includes
//----------------------------------------------------------------------------
#include "Jims_RobotClaw.h"
#include "StopWatch.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

//----------------------------------------------------------------------------
//  Namespace
//----------------------------------------------------------------------------
class DriveBase : public rclcpp::Node
{
  public:
    explicit DriveBase();
    virtual ~DriveBase() {}

  private:
    void timerCallback();
    int mCount = 0; 
    double mLeft = 0.0;
    double mRight = 0.0;
    StopWatch mMotorWatchdog = StopWatch(500);
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mTwistSub;
    void twistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);

    Jims_RobotClaw mRobotClaw = Jims_RobotClaw("/dev/ttyACM0",38400);

};
#endif
