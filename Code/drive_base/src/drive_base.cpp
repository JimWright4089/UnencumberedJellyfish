#include <chrono>
#include <memory>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
