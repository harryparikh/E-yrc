// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from linkattacher_msgs:srv/AttachLink.idl
// generated code does not contain a copyright notice

#ifndef LINKATTACHER_MSGS__SRV__DETAIL__ATTACH_LINK__BUILDER_HPP_
#define LINKATTACHER_MSGS__SRV__DETAIL__ATTACH_LINK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "linkattacher_msgs/srv/detail/attach_link__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace linkattacher_msgs
{

namespace srv
{

namespace builder
{

class Init_AttachLink_Request_data
{
public:
  Init_AttachLink_Request_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::linkattacher_msgs::srv::AttachLink_Request data(::linkattacher_msgs::srv::AttachLink_Request::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::linkattacher_msgs::srv::AttachLink_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::linkattacher_msgs::srv::AttachLink_Request>()
{
  return linkattacher_msgs::srv::builder::Init_AttachLink_Request_data();
}

}  // namespace linkattacher_msgs


namespace linkattacher_msgs
{

namespace srv
{

namespace builder
{

class Init_AttachLink_Response_message
{
public:
  explicit Init_AttachLink_Response_message(::linkattacher_msgs::srv::AttachLink_Response & msg)
  : msg_(msg)
  {}
  ::linkattacher_msgs::srv::AttachLink_Response message(::linkattacher_msgs::srv::AttachLink_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::linkattacher_msgs::srv::AttachLink_Response msg_;
};

class Init_AttachLink_Response_success
{
public:
  Init_AttachLink_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AttachLink_Response_message success(::linkattacher_msgs::srv::AttachLink_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_AttachLink_Response_message(msg_);
  }

private:
  ::linkattacher_msgs::srv::AttachLink_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::linkattacher_msgs::srv::AttachLink_Response>()
{
  return linkattacher_msgs::srv::builder::Init_AttachLink_Response_success();
}

}  // namespace linkattacher_msgs

#endif  // LINKATTACHER_MSGS__SRV__DETAIL__ATTACH_LINK__BUILDER_HPP_
