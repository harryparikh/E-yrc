// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hb_interfaces:msg/BotCmd.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD__BUILDER_HPP_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hb_interfaces/msg/detail/bot_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hb_interfaces
{

namespace msg
{

namespace builder
{

class Init_BotCmd_elbow
{
public:
  explicit Init_BotCmd_elbow(::hb_interfaces::msg::BotCmd & msg)
  : msg_(msg)
  {}
  ::hb_interfaces::msg::BotCmd elbow(::hb_interfaces::msg::BotCmd::_elbow_type arg)
  {
    msg_.elbow = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hb_interfaces::msg::BotCmd msg_;
};

class Init_BotCmd_base
{
public:
  explicit Init_BotCmd_base(::hb_interfaces::msg::BotCmd & msg)
  : msg_(msg)
  {}
  Init_BotCmd_elbow base(::hb_interfaces::msg::BotCmd::_base_type arg)
  {
    msg_.base = std::move(arg);
    return Init_BotCmd_elbow(msg_);
  }

private:
  ::hb_interfaces::msg::BotCmd msg_;
};

class Init_BotCmd_m3
{
public:
  explicit Init_BotCmd_m3(::hb_interfaces::msg::BotCmd & msg)
  : msg_(msg)
  {}
  Init_BotCmd_base m3(::hb_interfaces::msg::BotCmd::_m3_type arg)
  {
    msg_.m3 = std::move(arg);
    return Init_BotCmd_base(msg_);
  }

private:
  ::hb_interfaces::msg::BotCmd msg_;
};

class Init_BotCmd_m2
{
public:
  explicit Init_BotCmd_m2(::hb_interfaces::msg::BotCmd & msg)
  : msg_(msg)
  {}
  Init_BotCmd_m3 m2(::hb_interfaces::msg::BotCmd::_m2_type arg)
  {
    msg_.m2 = std::move(arg);
    return Init_BotCmd_m3(msg_);
  }

private:
  ::hb_interfaces::msg::BotCmd msg_;
};

class Init_BotCmd_m1
{
public:
  explicit Init_BotCmd_m1(::hb_interfaces::msg::BotCmd & msg)
  : msg_(msg)
  {}
  Init_BotCmd_m2 m1(::hb_interfaces::msg::BotCmd::_m1_type arg)
  {
    msg_.m1 = std::move(arg);
    return Init_BotCmd_m2(msg_);
  }

private:
  ::hb_interfaces::msg::BotCmd msg_;
};

class Init_BotCmd_id
{
public:
  Init_BotCmd_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BotCmd_m1 id(::hb_interfaces::msg::BotCmd::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_BotCmd_m1(msg_);
  }

private:
  ::hb_interfaces::msg::BotCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hb_interfaces::msg::BotCmd>()
{
  return hb_interfaces::msg::builder::Init_BotCmd_id();
}

}  // namespace hb_interfaces

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD__BUILDER_HPP_
