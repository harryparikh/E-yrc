// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hb_interfaces:msg/BotCmdArray.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__TRAITS_HPP_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hb_interfaces/msg/detail/bot_cmd_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'cmds'
#include "hb_interfaces/msg/detail/bot_cmd__traits.hpp"

namespace hb_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BotCmdArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmds
  {
    if (msg.cmds.size() == 0) {
      out << "cmds: []";
    } else {
      out << "cmds: [";
      size_t pending_items = msg.cmds.size();
      for (auto item : msg.cmds) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BotCmdArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmds
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cmds.size() == 0) {
      out << "cmds: []\n";
    } else {
      out << "cmds:\n";
      for (auto item : msg.cmds) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BotCmdArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace hb_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use hb_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hb_interfaces::msg::BotCmdArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  hb_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hb_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hb_interfaces::msg::BotCmdArray & msg)
{
  return hb_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hb_interfaces::msg::BotCmdArray>()
{
  return "hb_interfaces::msg::BotCmdArray";
}

template<>
inline const char * name<hb_interfaces::msg::BotCmdArray>()
{
  return "hb_interfaces/msg/BotCmdArray";
}

template<>
struct has_fixed_size<hb_interfaces::msg::BotCmdArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hb_interfaces::msg::BotCmdArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hb_interfaces::msg::BotCmdArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__TRAITS_HPP_
