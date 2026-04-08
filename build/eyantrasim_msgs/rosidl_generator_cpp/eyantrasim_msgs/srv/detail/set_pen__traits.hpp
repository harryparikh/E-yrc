// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eyantrasim_msgs:srv/SetPen.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__TRAITS_HPP_
#define EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "eyantrasim_msgs/srv/detail/set_pen__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace eyantrasim_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPen_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: r
  {
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << ", ";
  }

  // member: g
  {
    out << "g: ";
    rosidl_generator_traits::value_to_yaml(msg.g, out);
    out << ", ";
  }

  // member: b
  {
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: off
  {
    out << "off: ";
    rosidl_generator_traits::value_to_yaml(msg.off, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPen_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: r
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "r: ";
    rosidl_generator_traits::value_to_yaml(msg.r, out);
    out << "\n";
  }

  // member: g
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "g: ";
    rosidl_generator_traits::value_to_yaml(msg.g, out);
    out << "\n";
  }

  // member: b
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "b: ";
    rosidl_generator_traits::value_to_yaml(msg.b, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: off
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "off: ";
    rosidl_generator_traits::value_to_yaml(msg.off, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPen_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace eyantrasim_msgs

namespace rosidl_generator_traits
{

[[deprecated("use eyantrasim_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const eyantrasim_msgs::srv::SetPen_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  eyantrasim_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use eyantrasim_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const eyantrasim_msgs::srv::SetPen_Request & msg)
{
  return eyantrasim_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<eyantrasim_msgs::srv::SetPen_Request>()
{
  return "eyantrasim_msgs::srv::SetPen_Request";
}

template<>
inline const char * name<eyantrasim_msgs::srv::SetPen_Request>()
{
  return "eyantrasim_msgs/srv/SetPen_Request";
}

template<>
struct has_fixed_size<eyantrasim_msgs::srv::SetPen_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<eyantrasim_msgs::srv::SetPen_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<eyantrasim_msgs::srv::SetPen_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace eyantrasim_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetPen_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetPen_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetPen_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace eyantrasim_msgs

namespace rosidl_generator_traits
{

[[deprecated("use eyantrasim_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const eyantrasim_msgs::srv::SetPen_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  eyantrasim_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use eyantrasim_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const eyantrasim_msgs::srv::SetPen_Response & msg)
{
  return eyantrasim_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<eyantrasim_msgs::srv::SetPen_Response>()
{
  return "eyantrasim_msgs::srv::SetPen_Response";
}

template<>
inline const char * name<eyantrasim_msgs::srv::SetPen_Response>()
{
  return "eyantrasim_msgs/srv/SetPen_Response";
}

template<>
struct has_fixed_size<eyantrasim_msgs::srv::SetPen_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<eyantrasim_msgs::srv::SetPen_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<eyantrasim_msgs::srv::SetPen_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<eyantrasim_msgs::srv::SetPen>()
{
  return "eyantrasim_msgs::srv::SetPen";
}

template<>
inline const char * name<eyantrasim_msgs::srv::SetPen>()
{
  return "eyantrasim_msgs/srv/SetPen";
}

template<>
struct has_fixed_size<eyantrasim_msgs::srv::SetPen>
  : std::integral_constant<
    bool,
    has_fixed_size<eyantrasim_msgs::srv::SetPen_Request>::value &&
    has_fixed_size<eyantrasim_msgs::srv::SetPen_Response>::value
  >
{
};

template<>
struct has_bounded_size<eyantrasim_msgs::srv::SetPen>
  : std::integral_constant<
    bool,
    has_bounded_size<eyantrasim_msgs::srv::SetPen_Request>::value &&
    has_bounded_size<eyantrasim_msgs::srv::SetPen_Response>::value
  >
{
};

template<>
struct is_service<eyantrasim_msgs::srv::SetPen>
  : std::true_type
{
};

template<>
struct is_service_request<eyantrasim_msgs::srv::SetPen_Request>
  : std::true_type
{
};

template<>
struct is_service_response<eyantrasim_msgs::srv::SetPen_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__TRAITS_HPP_
