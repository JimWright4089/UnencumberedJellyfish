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
#include <math.h>

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
  mLengthOfWheel = mWheelDiameter * M_PI;
  mRadPerMeter = (1000.0 / mLengthOfWheel) * (2*M_PI);

  mRobotClaw = Jims_RobotClaw(mSerialPort.c_str(),mBaud);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  mOdomEncoderPub = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
  mTransformBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  mTwistSub = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&DriveBase::twistCallBack, this, std::placeholders::_1));
  mTimer = this->create_wall_timer(20ms, std::bind(&DriveBase::timerCallback, this));

  mRobotClaw.resetEncoders();
}

DriveBase::~DriveBase()
{
  mRobotClaw.setLeftSpeed(0);
  mRobotClaw.setRightSpeed(0);
}


void DriveBase::timerCallback()
{
  mRobotClaw.setLeftSpeed(500);
  mRobotClaw.setRightSpeed(500);
/*
  if(mMotorWatchdog.IsExpired())
  {
    mRobotClaw.setLeftMotor(0);
    mRobotClaw.setRightMotor(0);
  }
  else
  {
    mRobotClaw.setLeftSpeed(mLeft);
    mRobotClaw.setRightSpeed(mRight);
  }
*/  
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

  mLeft *= mMaxVelocity;
  mRight *= mMaxVelocity;

  mMotorWatchdog.Reset();
}

void DriveBase::publishOdometry()
{  // convert encoder readings to real world values and publish as Odometry
  static double previousTime = 0;
  double deltaTime = 0;
  static int count = 0;

  rclcpp::Time theTime = this->now();
  double currentTime = (double)theTime.nanoseconds()/(double)1000000000;
  deltaTime = currentTime - previousTime;
  previousTime = currentTime;

  if (deltaTime == 0.0) 
  {
    return;
  }

  double leftWheelEncoder = mRobotClaw.getLeftEncoder();
  double rightWheelEncoder = mRobotClaw.getRightEncoder();

  static double prevLeftWheelEncoder = 0.0;
  static double prevRightWheelEncoder = 0.0;

//  double leftWheelEncoder = prevLeftWheelEncoder+28.52661133;
//  double rightWheelEncoder = prevRightWheelEncoder+28.52661133;

  
  double deltaLeftWheelEncoderDist = 
        ((leftWheelEncoder-prevLeftWheelEncoder)/mEncoderTicks)*mLengthOfWheel;
  double deltaRightWheelEncoderDist = 
        ((rightWheelEncoder-prevRightWheelEncoder)/mEncoderTicks)*mLengthOfWheel;
/*  
  printf("le:%f ple:%f et:%f lw:%f dd:%f\n",
        leftWheelEncoder,
        prevLeftWheelEncoder,
        mEncoderTicks,
        mLengthOfWheel,
        deltaLeftWheelEncoderDist);
*/

  double velNet = (deltaRightWheelEncoderDist+deltaLeftWheelEncoderDist)/2;
  double velDiff = deltaRightWheelEncoderDist-deltaLeftWheelEncoderDist;

  mPose.position.x += (velNet * cos(mPose.orientation.z));
  mPose.position.y += (velNet * sin(mPose.orientation.z));
  
  double alpha = velDiff*mTraction*mRadPerMeter;
  mPose.orientation.z += alpha*deltaTime;

  prevLeftWheelEncoder = leftWheelEncoder;
  prevRightWheelEncoder = rightWheelEncoder;

  RCLCPP_INFO(this->get_logger(), "%d x : %f, y : %f theta : %f", 
        count,
        mPose.position.x, 
        mPose.position.y,
        mPose.orientation.z);
  count++;

  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  odom_msg->pose.pose.position.x = mPose.position.x;
  odom_msg->pose.pose.position.y = mPose.position.y;
  odom_msg->pose.pose.position.z = 0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, mPose.orientation.z);

  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

//  odom_msg->twist.twist.linear.x = translationalVelocity;
//  odom_msg->twist.twist.angular.z = rotationalVelocity;

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
