// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eyantrasim_msgs:srv/Kill.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__KILL__BUILDER_HPP_
#define EYANTRASIM_MSGS__SRV__DETAIL__KILL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eyantrasim_msgs/srv/detail/kill__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eyantrasim_msgs
{

namespace srv
{

namespace builder
{

class Init_Kill_Request_name
{
public:
  Init_Kill_Request_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::eyantrasim_msgs::srv::Kill_Request name(::eyantrasim_msgs::srv::Kill_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Kill_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eyantrasim_msgs::srv::Kill_Request>()
{
  return eyantrasim_msgs::srv::builder::Init_Kill_Request_name();
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
auto build<::eyantrasim_msgs::srv::Kill_Response>()
{
  return ::eyantrasim_msgs::srv::Kill_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace eyantrasim_msgs

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__KILL__BUILDER_HPP_
