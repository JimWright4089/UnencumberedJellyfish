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
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

//----------------------------------------------------------------------------
//  Namespace
//----------------------------------------------------------------------------
namespace unencumbered_jellyfish
{
  class drive_base : public rclcpp::Node
  {
    public:
      explicit drive_base();
      virtual ~drive_base() {}

    private:
  };
}
#endif
