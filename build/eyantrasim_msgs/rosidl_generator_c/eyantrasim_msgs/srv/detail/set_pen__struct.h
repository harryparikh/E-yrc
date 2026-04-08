// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from eyantrasim_msgs:srv/SetPen.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__STRUCT_H_
#define EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetPen in the package eyantrasim_msgs.
typedef struct eyantrasim_msgs__srv__SetPen_Request
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t width;
  uint8_t off;
} eyantrasim_msgs__srv__SetPen_Request;

// Struct for a sequence of eyantrasim_msgs__srv__SetPen_Request.
typedef struct eyantrasim_msgs__srv__SetPen_Request__Sequence
{
  eyantrasim_msgs__srv__SetPen_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eyantrasim_msgs__srv__SetPen_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetPen in the package eyantrasim_msgs.
typedef struct eyantrasim_msgs__srv__SetPen_Response
{
  uint8_t structure_needs_at_least_one_member;
} eyantrasim_msgs__srv__SetPen_Response;

// Struct for a sequence of eyantrasim_msgs__srv__SetPen_Response.
typedef struct eyantrasim_msgs__srv__SetPen_Response__Sequence
{
  eyantrasim_msgs__srv__SetPen_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} eyantrasim_msgs__srv__SetPen_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__SET_PEN__STRUCT_H_
