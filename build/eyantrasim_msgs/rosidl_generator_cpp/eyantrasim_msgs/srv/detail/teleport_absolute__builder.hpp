// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eyantrasim_msgs:srv/TeleportAbsolute.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_ABSOLUTE__BUILDER_HPP_
#define EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_ABSOLUTE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eyantrasim_msgs/srv/detail/teleport_absolute__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eyantrasim_msgs
{

namespace srv
{

namespace builder
{

class Init_TeleportAbsolute_Request_theta
{
public:
  explicit Init_TeleportAbsolute_Request_theta(::eyantrasim_msgs::srv::TeleportAbsolute_Request & msg)
  : msg_(msg)
  {}
  ::eyantrasim_msgs::srv::TeleportAbsolute_Request theta(::eyantrasim_msgs::srv::TeleportAbsolute_Request::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eyantrasim_msgs::srv::TeleportAbsolute_Request msg_;
};

class Init_TeleportAbsolute_Request_y
{
public:
  explicit Init_TeleportAbsolute_Request_y(::eyantrasim_msgs::srv::TeleportAbsolute_Request & msg)
  : msg_(msg)
  {}
  Init_TeleportAbsolute_Request_theta y(::eyantrasim_msgs::srv::TeleportAbsolute_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_TeleportAbsolute_Request_theta(msg_);
  }

private:
  ::eyantrasim_msgs::srv::TeleportAbsolute_Request msg_;
};

class Init_TeleportAbsolute_Request_x
{
public:
  Init_TeleportAbsolute_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TeleportAbsolute_Request_y x(::eyantrasim_msgs::srv::TeleportAbsolute_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_TeleportAbsolute_Request_y(msg_);
  }

private:
  ::eyantrasim_msgs::srv::TeleportAbsolute_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eyantrasim_msgs::srv::TeleportAbsolute_Request>()
{
  return eyantrasim_msgs::srv::builder::Init_TeleportAbsolute_Request_x();
}

}  // namespace eyantrasim_msgs


namespace eyantrasim_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eyantrasim_msgs::srv::TeleportAbsolute_Response>()
{
  return ::eyantrasim_msgs::srv::TeleportAbsolute_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace eyantrasim_msgs

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_ABSOLUTE__BUILDER_HPP_
