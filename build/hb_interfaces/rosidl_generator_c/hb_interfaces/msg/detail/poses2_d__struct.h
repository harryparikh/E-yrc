// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hb_interfaces:msg/Poses2D.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__POSES2_D__STRUCT_H_
#define HB_INTERFACES__MSG__DETAIL__POSES2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'poses'
#include "hb_interfaces/msg/detail/pose2_d__struct.h"

/// Struct defined in msg/Poses2D in the package hb_interfaces.
typedef struct hb_interfaces__msg__Poses2D
{
  hb_interfaces__msg__Pose2D__Sequence poses;
} hb_interfaces__msg__Poses2D;

// Struct for a sequence of hb_interfaces__msg__Poses2D.
typedef struct hb_interfaces__msg__Poses2D__Sequence
{
  hb_interfaces__msg__Poses2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hb_interfaces__msg__Poses2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HB_INTERFACES__MSG__DETAIL__POSES2_D__STRUCT_H_
