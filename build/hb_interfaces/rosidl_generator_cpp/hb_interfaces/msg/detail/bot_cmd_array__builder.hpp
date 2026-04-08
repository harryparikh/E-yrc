// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hb_interfaces:msg/BotCmdArray.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__BUILDER_HPP_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "hb_interfaces/msg/detail/bot_cmd_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace hb_interfaces
{

namespace msg
{

namespace builder
{

class Init_BotCmdArray_cmds
{
public:
  Init_BotCmdArray_cmds()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hb_interfaces::msg::BotCmdArray cmds(::hb_interfaces::msg::BotCmdArray::_cmds_type arg)
  {
    msg_.cmds = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hb_interfaces::msg::BotCmdArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hb_interfaces::msg::BotCmdArray>()
{
  return hb_interfaces::msg::builder::Init_BotCmdArray_cmds();
}

}  // namespace hb_interfaces

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__BUILDER_HPP_
