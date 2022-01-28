cmake_minimum_required(VERSION 3.5)
project(drive_base)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/robotclaw
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(drive_base src/drive_base.cpp 
    src/StopWatch.cpp
    src/robotclaw/Jims_RobotClaw.cpp 
    src/robotclaw/Jims_Serial.cpp)
ament_target_dependencies(drive_base rclcpp std_msgs geometry_msgs nav_msgs tf2_ros)

install(TARGETS
  drive_base
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
