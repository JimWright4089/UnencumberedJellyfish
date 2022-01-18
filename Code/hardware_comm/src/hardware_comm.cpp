#include "hardware_comm.hpp"

using namespace std::chrono_literals;

HardwareComm::HardwareComm()
  : Node("hardware_comm"), 
  mCount(0)
{
  mPubIMUUncompensated = this->create_publisher<sensor_msgs::msg::Imu>("imu_uncompensated", 10);  
  mTimer = this->create_wall_timer(20ms, std::bind(&HardwareComm::timerCallback, this));

  /* Initialise the sensor */
  if(!mBno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    RCLCPP_ERROR(this->get_logger(),"Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  sleep_for(milliseconds(1000));

  /* Display the current temperature */
  int8_t temp = mBno.getTemp();
  printf("Current Temperature: %d C\nn",temp);

  mBno.setExtCrystalUse(true);
  sleep_for(milliseconds(500));
}

void HardwareComm::timerCallback()
{
  sensor_msgs::msg::Imu message;

//  message.header.seq = mCount;
  mCount++;
  message.header.frame_id = "imu";

  imu::Quaternion quat = mBno.getQuat();
  message.orientation.x = quat.x();
  message.orientation.y = quat.y();
  message.orientation.z = quat.z();
  message.orientation.w = quat.w();
  message.orientation_covariance[0] = 0.0159;
  message.orientation_covariance[4] = 0.0159;
  message.orientation_covariance[8] = 0.0159;

  imu::Vector<3> euler = mBno.getVector(Jims_BNO055::VECTOR_GYROSCOPE);
  message.angular_velocity.x = euler.x();
  message.angular_velocity.y = euler.y();
  message.angular_velocity.z = euler.z();
  message.angular_velocity_covariance[0] = 0.04;
  message.angular_velocity_covariance[4] = 0.04;
  message.angular_velocity_covariance[8] = 0.04;

  euler = mBno.getVector(Jims_BNO055::VECTOR_LINEARACCEL);
  message.linear_acceleration.x = euler.x();
  message.linear_acceleration.y = euler.y();
  message.linear_acceleration.z = euler.z();
  message.linear_acceleration_covariance[0] = 0.017;
  message.linear_acceleration_covariance[4] = 0.017;
  message.linear_acceleration_covariance[8] = 0.017;

  mPubIMUUncompensated->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareComm>());
  rclcpp::shutdown();
  return 0;
}
