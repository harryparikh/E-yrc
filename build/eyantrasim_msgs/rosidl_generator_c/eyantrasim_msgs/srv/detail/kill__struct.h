// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eyantrasim_msgs:srv/Kill.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__KILL__STRUCT_H_
#define EYANTRASIM_MSGS__SRV__DETAIL__KILL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Kill in the package eyantrasim_msgs.
typedef struct eyantrasim_msgs__srv__Kill_Request
{
  rosidl_runtime_c__String name;
} eyantrasim_msgs__srv__Kill_Request;

// Struct for a sequence of eyantrasim_msgs__srv__Kill_Request.
typedef struct eyantrasim_msgs__srv__Kill_Request__Sequence
{
  eyantrasim_msgs__srv__Kill_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eyantrasim_msgs__srv__Kill_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/Kill in the package eyantrasim_msgs.
typedef struct eyantrasim_msgs__srv__Kill_Response
{
  uint8_t structure_needs_at_least_one_member;
} eyantrasim_msgs__srv__Kill_Response;

// Struct for a sequence of eyantrasim_msgs__srv__Kill_Response.
typedef struct eyantrasim_msgs__srv__Kill_Response__Sequence
{
  eyantrasim_msgs__srv__Kill_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eyantrasim_msgs__srv__Kill_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__KILL__STRUCT_H_
