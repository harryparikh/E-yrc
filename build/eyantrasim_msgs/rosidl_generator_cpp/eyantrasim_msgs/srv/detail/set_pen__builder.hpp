// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eyantrasim_msgs:srv/SetPen.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__BUILDER_HPP_
#define EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eyantrasim_msgs/srv/detail/set_pen__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eyantrasim_msgs
{

namespace srv
{

namespace builder
{

class Init_SetPen_Request_off
{
public:
  explicit Init_SetPen_Request_off(::eyantrasim_msgs::srv::SetPen_Request & msg)
  : msg_(msg)
  {}
  ::eyantrasim_msgs::srv::SetPen_Request off(::eyantrasim_msgs::srv::SetPen_Request::_off_type arg)
  {
    msg_.off = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eyantrasim_msgs::srv::SetPen_Request msg_;
};

class Init_SetPen_Request_width
{
public:
  explicit Init_SetPen_Request_width(::eyantrasim_msgs::srv::SetPen_Request & msg)
  : msg_(msg)
  {}
  Init_SetPen_Request_off width(::eyantrasim_msgs::srv::SetPen_Request::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_SetPen_Request_off(msg_);
  }

private:
  ::eyantrasim_msgs::srv::SetPen_Request msg_;
};

class Init_SetPen_Request_b
{
public:
  explicit Init_SetPen_Request_b(::eyantrasim_msgs::srv::SetPen_Request & msg)
  : msg_(msg)
  {}
  Init_SetPen_Request_width b(::eyantrasim_msgs::srv::SetPen_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_SetPen_Request_width(msg_);
  }

private:
  ::eyantrasim_msgs::srv::SetPen_Request msg_;
};

class Init_SetPen_Request_g
{
public:
  explicit Init_SetPen_Request_g(::eyantrasim_msgs::srv::SetPen_Request & msg)
  : msg_(msg)
  {}
  Init_SetPen_Request_b g(::eyantrasim_msgs::srv::SetPen_Request::_g_type arg)
  {
    msg_.g = std::move(arg);
    return Init_SetPen_Request_b(msg_);
  }

private:
  ::eyantrasim_msgs::srv::SetPen_Request msg_;
};

class Init_SetPen_Request_r
{
public:
  Init_SetPen_Request_r()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPen_Request_g r(::eyantrasim_msgs::srv::SetPen_Request::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_SetPen_Request_g(msg_);
  }

private:
  ::eyantrasim_msgs::srv::SetPen_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eyantrasim_msgs::srv::SetPen_Request>()
{
  return eyantrasim_msgs::srv::builder::Init_SetPen_Request_r();
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
auto build<::eyantrasim_msgs::srv::SetPen_Response>()
{
  return ::eyantrasim_msgs::srv::SetPen_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace eyantrasim_msgs

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__BUILDER_HPP_
