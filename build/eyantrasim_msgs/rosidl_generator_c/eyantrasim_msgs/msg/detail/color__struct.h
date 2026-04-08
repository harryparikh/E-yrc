// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eyantrasim_msgs:msg/Color.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__MSG__DETAIL__COLOR__STRUCT_H_
#define EYANTRASIM_MSGS__MSG__DETAIL__COLOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Color in the package eyantrasim_msgs.
typedef struct eyantrasim_msgs__msg__Color
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} eyantrasim_msgs__msg__Color;

// Struct for a sequence of eyantrasim_msgs__msg__Color.
typedef struct eyantrasim_msgs__msg__Color__Sequence
{
  eyantrasim_msgs__msg__Color * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eyantrasim_msgs__msg__Color__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EYANTRASIM_MSGS__MSG__DETAIL__COLOR__STRUCT_H_
