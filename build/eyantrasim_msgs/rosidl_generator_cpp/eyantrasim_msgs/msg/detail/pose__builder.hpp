// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eyantrasim_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
#define EYANTRASIM_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eyantrasim_msgs/msg/detail/pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eyantrasim_msgs
{

namespace msg
{

namespace builder
{

class Init_Pose_angular_velocity
{
public:
  explicit Init_Pose_angular_velocity(::eyantrasim_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  ::eyantrasim_msgs::msg::Pose angular_velocity(::eyantrasim_msgs::msg::Pose::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eyantrasim_msgs::msg::Pose msg_;
};

class Init_Pose_linear_velocity
{
public:
  explicit Init_Pose_linear_velocity(::eyantrasim_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_angular_velocity linear_velocity(::eyantrasim_msgs::msg::Pose::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_Pose_angular_velocity(msg_);
  }

private:
  ::eyantrasim_msgs::msg::Pose msg_;
};

class Init_Pose_theta
{
public:
  explicit Init_Pose_theta(::eyantrasim_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_linear_velocity theta(::eyantrasim_msgs::msg::Pose::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_Pose_linear_velocity(msg_);
  }

private:
  ::eyantrasim_msgs::msg::Pose msg_;
};

class Init_Pose_y
{
public:
  explicit Init_Pose_y(::eyantrasim_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_theta y(::eyantrasim_msgs::msg::Pose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Pose_theta(msg_);
  }

private:
  ::eyantrasim_msgs::msg::Pose msg_;
};

class Init_Pose_x
{
public:
  Init_Pose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose_y x(::eyantrasim_msgs::msg::Pose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Pose_y(msg_);
  }

private:
  ::eyantrasim_msgs::msg::Pose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::eyantrasim_msgs::msg::Pose>()
{
  return eyantrasim_msgs::msg::builder::Init_Pose_x();
}

}  // namespace eyantrasim_msgs

#endif  // EYANTRASIM_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
