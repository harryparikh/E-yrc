// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eyantrasim_msgs:srv/TeleportRelative.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_RELATIVE__BUILDER_HPP_
#define EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_RELATIVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eyantrasim_msgs/srv/detail/teleport_relative__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eyantrasim_msgs
{

namespace srv
{

namespace builder
{

class Init_TeleportRelative_Request_angular
{
public:
  explicit Init_TeleportRelative_Request_angular(::eyantrasim_msgs::srv::TeleportRelative_Request & msg)
  : msg_(msg)
  {}
  ::eyantrasim_msgs::srv::TeleportRelative_Request angular(::eyantrasim_msgs::srv::TeleportRelative_Request::_angular_type arg)
  {
    msg_.angular = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eyantrasim_msgs::srv::TeleportRelative_Request msg_;
};

class Init_TeleportRelative_Request_linear
{
public:
  Init_TeleportRelative_Request_linear()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TeleportRelative_Request_angular linear(::eyantrasim_msgs::srv::TeleportRelative_Request::_linear_type arg)
  {
    msg_.linear = std::move(arg);
    return Init_TeleportRelative_Request_angular(msg_);
  }

private:
  ::eyantrasim_msgs::srv::TeleportRelative_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eyantrasim_msgs::srv::TeleportRelative_Request>()
{
  return eyantrasim_msgs::srv::builder::Init_TeleportRelative_Request_linear();
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
auto build<::eyantrasim_msgs::srv::TeleportRelative_Response>()
{
  return ::eyantrasim_msgs::srv::TeleportRelative_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace eyantrasim_msgs

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_RELATIVE__BUILDER_HPP_
