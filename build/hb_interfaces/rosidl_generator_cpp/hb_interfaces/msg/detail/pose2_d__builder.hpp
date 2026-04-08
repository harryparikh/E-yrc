// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hb_interfaces:msg/Pose2D.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__POSE2_D__BUILDER_HPP_
#define HB_INTERFACES__MSG__DETAIL__POSE2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hb_interfaces/msg/detail/pose2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hb_interfaces
{

namespace msg
{

namespace builder
{

class Init_Pose2D_w
{
public:
  explicit Init_Pose2D_w(::hb_interfaces::msg::Pose2D & msg)
  : msg_(msg)
  {}
  ::hb_interfaces::msg::Pose2D w(::hb_interfaces::msg::Pose2D::_w_type arg)
  {
    msg_.w = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hb_interfaces::msg::Pose2D msg_;
};

class Init_Pose2D_y
{
public:
  explicit Init_Pose2D_y(::hb_interfaces::msg::Pose2D & msg)
  : msg_(msg)
  {}
  Init_Pose2D_w y(::hb_interfaces::msg::Pose2D::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Pose2D_w(msg_);
  }

private:
  ::hb_interfaces::msg::Pose2D msg_;
};

class Init_Pose2D_x
{
public:
  explicit Init_Pose2D_x(::hb_interfaces::msg::Pose2D & msg)
  : msg_(msg)
  {}
  Init_Pose2D_y x(::hb_interfaces::msg::Pose2D::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Pose2D_y(msg_);
  }

private:
  ::hb_interfaces::msg::Pose2D msg_;
};

class Init_Pose2D_id
{
public:
  Init_Pose2D_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose2D_x id(::hb_interfaces::msg::Pose2D::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Pose2D_x(msg_);
  }

private:
  ::hb_interfaces::msg::Pose2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hb_interfaces::msg::Pose2D>()
{
  return hb_interfaces::msg::builder::Init_Pose2D_id();
}

}  // namespace hb_interfaces

#endif  // HB_INTERFACES__MSG__DETAIL__POSE2_D__BUILDER_HPP_
