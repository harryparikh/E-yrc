// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hb_interfaces:msg/BotCmd.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD__TRAITS_HPP_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hb_interfaces/msg/detail/bot_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace hb_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BotCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: m1
  {
    out << "m1: ";
    rosidl_generator_traits::value_to_yaml(msg.m1, out);
    out << ", ";
  }

  // member: m2
  {
    out << "m2: ";
    rosidl_generator_traits::value_to_yaml(msg.m2, out);
    out << ", ";
  }

  // member: m3
  {
    out << "m3: ";
    rosidl_generator_traits::value_to_yaml(msg.m3, out);
    out << ", ";
  }

  // member: base
  {
    out << "base: ";
    rosidl_generator_traits::value_to_yaml(msg.base, out);
    out << ", ";
  }

  // member: elbow
  {
    out << "elbow: ";
    rosidl_generator_traits::value_to_yaml(msg.elbow, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BotCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: m1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m1: ";
    rosidl_generator_traits::value_to_yaml(msg.m1, out);
    out << "\n";
  }

  // member: m2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m2: ";
    rosidl_generator_traits::value_to_yaml(msg.m2, out);
    out << "\n";
  }

  // member: m3
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "m3: ";
    rosidl_generator_traits::value_to_yaml(msg.m3, out);
    out << "\n";
  }

  // member: base
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "base: ";
    rosidl_generator_traits::value_to_yaml(msg.base, out);
    out << "\n";
  }

  // member: elbow
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "elbow: ";
    rosidl_generator_traits::value_to_yaml(msg.elbow, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BotCmd & msg, bool use_flow_style = false)
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
  const hb_interfaces::msg::BotCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  hb_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hb_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hb_interfaces::msg::BotCmd & msg)
{
  return hb_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hb_interfaces::msg::BotCmd>()
{
  return "hb_interfaces::msg::BotCmd";
}

template<>
inline const char * name<hb_interfaces::msg::BotCmd>()
{
  return "hb_interfaces/msg/BotCmd";
}

template<>
struct has_fixed_size<hb_interfaces::msg::BotCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hb_interfaces::msg::BotCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hb_interfaces::msg::BotCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD__TRAITS_HPP_
