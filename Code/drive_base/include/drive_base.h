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
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>

//----------------------------------------------------------------------------
//  Namespace
//----------------------------------------------------------------------------
class DriveBase : public rclcpp::Node
{
  public:
    explicit DriveBase();
    ~DriveBase();

  private:
    int mCount = 0; 
    double mLeft = 0.0;
    double mRight = 0.0;

    string mSerialPort = "X";
    uint16_t mBaud = 0;
    double   mWheelDiameter = 0.0;
    double   mEncoderTicks = 0.0;
    double   mMaxVelocity = 0.0;
    double   mDistBetweenWheels = 0.0;
    double   mLengthOfWheel = 0.0;
    double   mRadPerMeter = 4.17;
    double   mTraction = 0.99;
    bool     mPublishTransform = false;
    geometry_msgs::msg::Pose mPose;

    StopWatch mMotorWatchdog = StopWatch(500);
    rclcpp::TimerBase::SharedPtr mTimer;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr mTwistSub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mOdomEncoderPub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTransformBroadcaster;
    void twistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg);

    Jims_RobotClaw mRobotClaw = Jims_RobotClaw("/dev/ttyACM0",38400);

    void timerCallback();
    void publishOdometry();
};
#endif
