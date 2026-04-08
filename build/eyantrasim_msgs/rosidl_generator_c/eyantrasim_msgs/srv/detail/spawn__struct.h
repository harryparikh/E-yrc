// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eyantrasim_msgs:srv/Spawn.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__STRUCT_H_
#define EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__STRUCT_H_

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

/// Struct defined in srv/Spawn in the package eyantrasim_msgs.
typedef struct eyantrasim_msgs__srv__Spawn_Request
{
  float x;
  float y;
  float theta;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t width;
  /// Optional. Index of the image to use (0-2). If -1 or out of range, will use random selection
  int32_t image_index;
  /// Optional.  A unique name will be created and returned if this is empty
  rosidl_runtime_c__String name;
} eyantrasim_msgs__srv__Spawn_Request;

// Struct for a sequence of eyantrasim_msgs__srv__Spawn_Request.
typedef struct eyantrasim_msgs__srv__Spawn_Request__Sequence
{
  eyantrasim_msgs__srv__Spawn_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eyantrasim_msgs__srv__Spawn_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'name'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Spawn in the package eyantrasim_msgs.
typedef struct eyantrasim_msgs__srv__Spawn_Response
{
  rosidl_runtime_c__String name;
} eyantrasim_msgs__srv__Spawn_Response;

// Struct for a sequence of eyantrasim_msgs__srv__Spawn_Response.
typedef struct eyantrasim_msgs__srv__Spawn_Response__Sequence
{
  eyantrasim_msgs__srv__Spawn_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eyantrasim_msgs__srv__Spawn_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__STRUCT_H_
