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
#include "drive_base.h"

DriveBase::DriveBase()
  : Node("drive_base")
{
//    publisher_ = this->create_publisher<std_msgs::msg::Float64>("sinValue", 10);
  mTwistSub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&DriveBase::twistCallBack, this, std::placeholders::_1));
//      "/cmd_vel", 10, std::bind(&DriveBase::twistCallBack, this, 10));
  mTimer = this->create_wall_timer(20ms, std::bind(&DriveBase::timerCallback, this));
}

void DriveBase::timerCallback()
{
  if(mMotorWatchdog.IsExpired())
  {
    mRobotClaw.setLeftMotor(0);
    mRobotClaw.setRightMotor(0);
  }
  else
  {
    mRobotClaw.setLeftMotor(mLeft);
    mRobotClaw.setRightMotor(mRight);
  }
}

void DriveBase::twistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{

  printf("DriveBase::twistCallBack\n");
  mLeft = msg->linear.x - msg->angular.z;
  mRight = msg->linear.x - msg->angular.z;

  if(mLeft > 1.0) mLeft = 1.0;
  if(mRight > 1.0) mRight = 1.0;
  if(mLeft < -1.0) mLeft = -1.0;
  if(mRight < -1.0) mRight = -1.0;

  mMotorWatchdog.Reset();
}

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
  rclcpp::spin(std::make_shared<DriveBase>());
  rclcpp::shutdown();
  return 0;
}
