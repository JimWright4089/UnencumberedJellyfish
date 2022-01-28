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
#include <tf2/LinearMath/Quaternion.h>

DriveBase::DriveBase()
  : Node("drive_base")
{
  RCLCPP_INFO(this->get_logger(), "drive_base startup");
  RCLCPP_INFO(this->get_logger(), "Params:");

  this->declare_parameter<std::string>("serial_port", "world");
  this->declare_parameter<int16_t>("baud", 10);
  this->declare_parameter<double>("wheel_diameter", 10.01);
  this->declare_parameter<double>("encoder_ticks", 20.02);
  this->declare_parameter<double>("max_velocity", 30.03);
  this->declare_parameter<double>("dist_between_wheels", 40.04);
  this->declare_parameter<bool>("publish_transform", false);

  this->get_parameter("serial_port", mSerialPort);
  this->get_parameter("baud", mBaud);
  this->get_parameter("wheel_diameter", mWheelDiameter);
  this->get_parameter("encoder_ticks", mEncoderTicks);
  this->get_parameter("max_velocity", mMaxVelocity);
  this->get_parameter("dist_between_wheels", mDistBetweenWheels);
  this->get_parameter("publish_transform", mPublishTransform);

  RCLCPP_INFO(this->get_logger(), "  mSerialPort   :%s", mSerialPort.c_str());
  RCLCPP_INFO(this->get_logger(), "  mBaud         :%d", mBaud);
  RCLCPP_INFO(this->get_logger(), "  mWheelDiameter:%f", mWheelDiameter);
  RCLCPP_INFO(this->get_logger(), "  mEncoderTicks :%f", mEncoderTicks);
  RCLCPP_INFO(this->get_logger(), "  mMaxVelocity  :%f", mMaxVelocity);
  RCLCPP_INFO(this->get_logger(), "  mDistBetweenWheels:%f", mDistBetweenWheels);
  RCLCPP_INFO(this->get_logger(), "  mPublishTransform:%d",mPublishTransform);

  mPose.position.x = 0;
  mPose.position.y = 0;
  mPose.orientation.z = 0; 

  mRobotClaw = Jims_RobotClaw(mSerialPort.c_str(),mBaud);
  mRobotClaw.setWheelDiameter(mWheelDiameter);
  mRobotClaw.setTicksPerRev(mEncoderTicks);
  mRobotClaw.setMaxSpeed(mMaxVelocity);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  mOdomEncoderPub = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  mTransformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  mTwistSub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&DriveBase::twistCallBack, this, std::placeholders::_1));
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
  publishOdometry();
}

void DriveBase::twistCallBack(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  mLeft = msg->linear.x - msg->angular.z;
  mRight = msg->linear.x - msg->angular.z;

  if(mLeft > 1.0) mLeft = 1.0;
  if(mRight > 1.0) mRight = 1.0;
  if(mLeft < -1.0) mLeft = -1.0;
  if(mRight < -1.0) mRight = -1.0;

  mMotorWatchdog.Reset();
}

void DriveBase::publishOdometry()
{  // convert encoder readings to real world values and publish as Odometry
  static double previousTime = 0;
  double deltaTime = 0;

  rclcpp::Time theTime = this->now();
  double currentTime = (double)theTime.nanoseconds()/(double)1000000000;
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;

  if (deltaTime == 0.0) 
  {
    return;
  }

  // rotation value of wheel [rad]
//  double leftWheelRad = mRobotClaw.getLeftSpeedAsRad();
//  double rightWheelRad = mRobotClaw.getRightSpeedAsRad();
  double leftWheelRad = 3.1415;
  double rightWheelRad = 3.1415;

  double delta_s = 0.0;
  double delta_theta = 0.0;

  double theta = 0.0;
  static double last_theta = 0.0;

  if (std::isnan(leftWheelRad)) 
  {
    leftWheelRad = 0.0;
  }

  if (std::isnan(rightWheelRad)) 
  {
    rightWheelRad = 0.0;
  }

  delta_s = (mWheelDiameter/2) * (rightWheelRad + leftWheelRad) / 2.0;
  theta = (mWheelDiameter/2) * (rightWheelRad - leftWheelRad) / mDistBetweenWheels;
  delta_theta = theta;

  // compute odometric pose
  mPose.position.x += delta_s * cos(mPose.orientation.z + (delta_theta / 2.0));
  mPose.position.y += delta_s * sin(mPose.orientation.z + (delta_theta / 2.0));
  mPose.orientation.z += delta_theta;

  RCLCPP_INFO(this->get_logger(), "x : %f, y : %f theta : %f", 
        mPose.position.x, 
        mPose.position.y,
        mPose.orientation.z);

  // compute odometric instantaneouse velocity
  double translationalVelocity = delta_s / deltaTime;
  double rotationalVelocity = delta_theta / deltaTime;

  last_theta = theta;

  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->header.frame_id = "odom";
  odom_msg->child_frame_id = "base_link";
//  odom_msg->header.stamp = this->get_clock().now();

  odom_msg->pose.pose.position.x = mPose.position.x;
  odom_msg->pose.pose.position.y = mPose.position.y;
  odom_msg->pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, mPose.orientation.z);

  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  odom_msg->twist.twist.linear.x = translationalVelocity;
  odom_msg->twist.twist.angular.z = rotationalVelocity;

  geometry_msgs::msg::TransformStamped odomTransform;

  odomTransform.transform.translation.x = odom_msg->pose.pose.position.x;
  odomTransform.transform.translation.y = odom_msg->pose.pose.position.y;
  odomTransform.transform.translation.z = odom_msg->pose.pose.position.z;
  odomTransform.transform.rotation = odom_msg->pose.pose.orientation;

  odomTransform.header.frame_id = odom_msg->header.frame_id;
  odomTransform.child_frame_id = odom_msg->child_frame_id;
//  odomTransform.header.stamp = odom_msg->header.stamp;

  mOdomEncoderPub->publish(std::move(odom_msg));

  if (mPublishTransform) 
  {
    mTransformBroadcaster->sendTransform(odomTransform);
  }
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
