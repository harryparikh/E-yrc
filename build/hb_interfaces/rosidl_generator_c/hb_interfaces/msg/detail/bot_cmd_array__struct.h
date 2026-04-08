// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hb_interfaces:msg/BotCmdArray.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__STRUCT_H_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cmds'
#include "hb_interfaces/msg/detail/bot_cmd__struct.h"

/// Struct defined in msg/BotCmdArray in the package hb_interfaces.
typedef struct hb_interfaces__msg__BotCmdArray
{
  hb_interfaces__msg__BotCmd__Sequence cmds;
} hb_interfaces__msg__BotCmdArray;

// Struct for a sequence of hb_interfaces__msg__BotCmdArray.
typedef struct hb_interfaces__msg__BotCmdArray__Sequence
{
  hb_interfaces__msg__BotCmdArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hb_interfaces__msg__BotCmdArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__STRUCT_H_
