// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hb_interfaces:msg/BotCmd.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD__STRUCT_H_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/BotCmd in the package hb_interfaces.
typedef struct hb_interfaces__msg__BotCmd
{
  int32_t id;
  float m1;
  float m2;
  float m3;
  float base;
  float elbow;
} hb_interfaces__msg__BotCmd;

// Struct for a sequence of hb_interfaces__msg__BotCmd.
typedef struct hb_interfaces__msg__BotCmd__Sequence
{
  hb_interfaces__msg__BotCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hb_interfaces__msg__BotCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD__STRUCT_H_
