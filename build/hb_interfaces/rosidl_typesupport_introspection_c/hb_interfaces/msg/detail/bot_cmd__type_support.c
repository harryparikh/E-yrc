// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hb_interfaces:msg/BotCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hb_interfaces/msg/detail/bot_cmd__rosidl_typesupport_introspection_c.h"
#include "hb_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hb_interfaces/msg/detail/bot_cmd__functions.h"
#include "hb_interfaces/msg/detail/bot_cmd__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hb_interfaces__msg__BotCmd__init(message_memory);
}

void hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_fini_function(void * message_memory)
{
  hb_interfaces__msg__BotCmd__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_message_member_array[6] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hb_interfaces__msg__BotCmd, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "m1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hb_interfaces__msg__BotCmd, m1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "m2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hb_interfaces__msg__BotCmd, m2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "m3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hb_interfaces__msg__BotCmd, m3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "base",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hb_interfaces__msg__BotCmd, base),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "elbow",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hb_interfaces__msg__BotCmd, elbow),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_message_members = {
  "hb_interfaces__msg",  // message namespace
  "BotCmd",  // message name
  6,  // number of fields
  sizeof(hb_interfaces__msg__BotCmd),
  hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_message_member_array,  // message members
  hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_message_type_support_handle = {
  0,
  &hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hb_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hb_interfaces, msg, BotCmd)() {
  if (!hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_message_type_support_handle.typesupport_identifier) {
    hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hb_interfaces__msg__BotCmd__rosidl_typesupport_introspection_c__BotCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
