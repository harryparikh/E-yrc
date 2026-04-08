// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from eyantrasim_msgs:msg/Color.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__MSG__DETAIL__COLOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define EYANTRASIM_MSGS__MSG__DETAIL__COLOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "eyantrasim_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "eyantrasim_msgs/msg/detail/color__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace eyantrasim_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eyantrasim_msgs
cdr_serialize(
  const eyantrasim_msgs::msg::Color & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eyantrasim_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  eyantrasim_msgs::msg::Color & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eyantrasim_msgs
get_serialized_size(
  const eyantrasim_msgs::msg::Color & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eyantrasim_msgs
max_serialized_size_Color(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace eyantrasim_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_eyantrasim_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, eyantrasim_msgs, msg, Color)();

#ifdef __cplusplus
}
#endif

#endif  // EYANTRASIM_MSGS__MSG__DETAIL__COLOR__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
