// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from eyantrasim_msgs:srv/Spawn.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__BUILDER_HPP_
#define EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "eyantrasim_msgs/srv/detail/spawn__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace eyantrasim_msgs
{

namespace srv
{

namespace builder
{

class Init_Spawn_Request_name
{
public:
  explicit Init_Spawn_Request_name(::eyantrasim_msgs::srv::Spawn_Request & msg)
  : msg_(msg)
  {}
  ::eyantrasim_msgs::srv::Spawn_Request name(::eyantrasim_msgs::srv::Spawn_Request::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

class Init_Spawn_Request_image_index
{
public:
  explicit Init_Spawn_Request_image_index(::eyantrasim_msgs::srv::Spawn_Request & msg)
  : msg_(msg)
  {}
  Init_Spawn_Request_name image_index(::eyantrasim_msgs::srv::Spawn_Request::_image_index_type arg)
  {
    msg_.image_index = std::move(arg);
    return Init_Spawn_Request_name(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

class Init_Spawn_Request_width
{
public:
  explicit Init_Spawn_Request_width(::eyantrasim_msgs::srv::Spawn_Request & msg)
  : msg_(msg)
  {}
  Init_Spawn_Request_image_index width(::eyantrasim_msgs::srv::Spawn_Request::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Spawn_Request_image_index(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

class Init_Spawn_Request_b
{
public:
  explicit Init_Spawn_Request_b(::eyantrasim_msgs::srv::Spawn_Request & msg)
  : msg_(msg)
  {}
  Init_Spawn_Request_width b(::eyantrasim_msgs::srv::Spawn_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return Init_Spawn_Request_width(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

class Init_Spawn_Request_g
{
public:
  explicit Init_Spawn_Request_g(::eyantrasim_msgs::srv::Spawn_Request & msg)
  : msg_(msg)
  {}
  Init_Spawn_Request_b g(::eyantrasim_msgs::srv::Spawn_Request::_g_type arg)
  {
    msg_.g = std::move(arg);
    return Init_Spawn_Request_b(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

class Init_Spawn_Request_r
{
public:
  explicit Init_Spawn_Request_r(::eyantrasim_msgs::srv::Spawn_Request & msg)
  : msg_(msg)
  {}
  Init_Spawn_Request_g r(::eyantrasim_msgs::srv::Spawn_Request::_r_type arg)
  {
    msg_.r = std::move(arg);
    return Init_Spawn_Request_g(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

class Init_Spawn_Request_theta
{
public:
  explicit Init_Spawn_Request_theta(::eyantrasim_msgs::srv::Spawn_Request & msg)
  : msg_(msg)
  {}
  Init_Spawn_Request_r theta(::eyantrasim_msgs::srv::Spawn_Request::_theta_type arg)
  {
    msg_.theta = std::move(arg);
    return Init_Spawn_Request_r(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

class Init_Spawn_Request_y
{
public:
  explicit Init_Spawn_Request_y(::eyantrasim_msgs::srv::Spawn_Request & msg)
  : msg_(msg)
  {}
  Init_Spawn_Request_theta y(::eyantrasim_msgs::srv::Spawn_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Spawn_Request_theta(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

class Init_Spawn_Request_x
{
public:
  Init_Spawn_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Spawn_Request_y x(::eyantrasim_msgs::srv::Spawn_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Spawn_Request_y(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eyantrasim_msgs::srv::Spawn_Request>()
{
  return eyantrasim_msgs::srv::builder::Init_Spawn_Request_x();
}

}  // namespace eyantrasim_msgs


namespace eyantrasim_msgs
{

namespace srv
{

namespace builder
{

class Init_Spawn_Response_name
{
public:
  Init_Spawn_Response_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::eyantrasim_msgs::srv::Spawn_Response name(::eyantrasim_msgs::srv::Spawn_Response::_name_type arg)
  {
    msg_.name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::eyantrasim_msgs::srv::Spawn_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::eyantrasim_msgs::srv::Spawn_Response>()
{
  return eyantrasim_msgs::srv::builder::Init_Spawn_Response_name();
}

}  // namespace eyantrasim_msgs

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__BUILDER_HPP_
