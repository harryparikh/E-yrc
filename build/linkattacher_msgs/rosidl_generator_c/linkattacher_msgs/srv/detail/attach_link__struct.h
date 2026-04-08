// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from linkattacher_msgs:srv/AttachLink.idl
// generated code does not contain a copyright notice

#ifndef LINKATTACHER_MSGS__SRV__DETAIL__ATTACH_LINK__STRUCT_H_
#define LINKATTACHER_MSGS__SRV__DETAIL__ATTACH_LINK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/AttachLink in the package linkattacher_msgs.
typedef struct linkattacher_msgs__srv__AttachLink_Request
{
  /// name of the model and link respectively
  rosidl_runtime_c__String data;
} linkattacher_msgs__srv__AttachLink_Request;

// Struct for a sequence of linkattacher_msgs__srv__AttachLink_Request.
typedef struct linkattacher_msgs__srv__AttachLink_Request__Sequence
{
  linkattacher_msgs__srv__AttachLink_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} linkattacher_msgs__srv__AttachLink_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/AttachLink in the package linkattacher_msgs.
typedef struct linkattacher_msgs__srv__AttachLink_Response
{
  /// result of the attach operation
  bool success;
  /// Error or success message
  rosidl_runtime_c__String message;
} linkattacher_msgs__srv__AttachLink_Response;

// Struct for a sequence of linkattacher_msgs__srv__AttachLink_Response.
typedef struct linkattacher_msgs__srv__AttachLink_Response__Sequence
{
  linkattacher_msgs__srv__AttachLink_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} linkattacher_msgs__srv__AttachLink_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LINKATTACHER_MSGS__SRV__DETAIL__ATTACH_LINK__STRUCT_H_
