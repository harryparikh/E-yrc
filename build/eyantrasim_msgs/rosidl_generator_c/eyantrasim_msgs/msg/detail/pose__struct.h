// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eyantrasim_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__MSG__DETAIL__POSE__STRUCT_H_
#define EYANTRASIM_MSGS__MSG__DETAIL__POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Pose in the package eyantrasim_msgs.
typedef struct eyantrasim_msgs__msg__Pose
{
  float x;
  float y;
  float theta;
  float linear_velocity;
  float angular_velocity;
} eyantrasim_msgs__msg__Pose;

// Struct for a sequence of eyantrasim_msgs__msg__Pose.
typedef struct eyantrasim_msgs__msg__Pose__Sequence
{
  eyantrasim_msgs__msg__Pose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eyantrasim_msgs__msg__Pose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EYANTRASIM_MSGS__MSG__DETAIL__POSE__STRUCT_H_
