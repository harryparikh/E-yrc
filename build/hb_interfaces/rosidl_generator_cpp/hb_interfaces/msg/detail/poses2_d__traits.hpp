// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hb_interfaces:msg/Poses2D.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__POSES2_D__TRAITS_HPP_
#define HB_INTERFACES__MSG__DETAIL__POSES2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hb_interfaces/msg/detail/poses2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'poses'
#include "hb_interfaces/msg/detail/pose2_d__traits.hpp"

namespace hb_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Poses2D & msg,
  std::ostream & out)
{
  out << "{";
  // member: poses
  {
    if (msg.poses.size() == 0) {
      out << "poses: []";
    } else {
      out << "poses: [";
      size_t pending_items = msg.poses.size();
      for (auto item : msg.poses) {
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
  const Poses2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Poses2D & msg, bool use_flow_style = false)
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
  const hb_interfaces::msg::Poses2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  hb_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hb_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const hb_interfaces::msg::Poses2D & msg)
{
  return hb_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hb_interfaces::msg::Poses2D>()
{
  return "hb_interfaces::msg::Poses2D";
}

template<>
inline const char * name<hb_interfaces::msg::Poses2D>()
{
  return "hb_interfaces/msg/Poses2D";
}

template<>
struct has_fixed_size<hb_interfaces::msg::Poses2D>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hb_interfaces::msg::Poses2D>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hb_interfaces::msg::Poses2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HB_INTERFACES__MSG__DETAIL__POSES2_D__TRAITS_HPP_
