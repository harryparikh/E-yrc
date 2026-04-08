// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hb_interfaces:msg/Pose2D.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__POSE2_D__STRUCT_H_
#define HB_INTERFACES__MSG__DETAIL__POSE2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Pose2D in the package hb_interfaces.
typedef struct hb_interfaces__msg__Pose2D
{
  int32_t id;
  float x;
  float y;
  float w;
} hb_interfaces__msg__Pose2D;

// Struct for a sequence of hb_interfaces__msg__Pose2D.
typedef struct hb_interfaces__msg__Pose2D__Sequence
{
  hb_interfaces__msg__Pose2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hb_interfaces__msg__Pose2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HB_INTERFACES__MSG__DETAIL__POSE2_D__STRUCT_H_
