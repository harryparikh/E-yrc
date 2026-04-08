// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hb_interfaces:msg/Poses2D.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hb_interfaces/msg/detail/poses2_d__rosidl_typesupport_introspection_c.h"
#include "hb_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hb_interfaces/msg/detail/poses2_d__functions.h"
#include "hb_interfaces/msg/detail/poses2_d__struct.h"


// Include directives for member types
// Member `poses`
#include "hb_interfaces/msg/pose2_d.h"
// Member `poses`
#include "hb_interfaces/msg/detail/pose2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hb_interfaces__msg__Poses2D__init(message_memory);
}

void hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_fini_function(void * message_memory)
{
  hb_interfaces__msg__Poses2D__fini(message_memory);
}

size_t hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__size_function__Poses2D__poses(
  const void * untyped_member)
{
  const hb_interfaces__msg__Pose2D__Sequence * member =
    (const hb_interfaces__msg__Pose2D__Sequence *)(untyped_member);
  return member->size;
}

const void * hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__get_const_function__Poses2D__poses(
  const void * untyped_member, size_t index)
{
  const hb_interfaces__msg__Pose2D__Sequence * member =
    (const hb_interfaces__msg__Pose2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void * hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__get_function__Poses2D__poses(
  void * untyped_member, size_t index)
{
  hb_interfaces__msg__Pose2D__Sequence * member =
    (hb_interfaces__msg__Pose2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__fetch_function__Poses2D__poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const hb_interfaces__msg__Pose2D * item =
    ((const hb_interfaces__msg__Pose2D *)
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__get_const_function__Poses2D__poses(untyped_member, index));
  hb_interfaces__msg__Pose2D * value =
    (hb_interfaces__msg__Pose2D *)(untyped_value);
  *value = *item;
}

void hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__assign_function__Poses2D__poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  hb_interfaces__msg__Pose2D * item =
    ((hb_interfaces__msg__Pose2D *)
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__get_function__Poses2D__poses(untyped_member, index));
  const hb_interfaces__msg__Pose2D * value =
    (const hb_interfaces__msg__Pose2D *)(untyped_value);
  *item = *value;
}

bool hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__resize_function__Poses2D__poses(
  void * untyped_member, size_t size)
{
  hb_interfaces__msg__Pose2D__Sequence * member =
    (hb_interfaces__msg__Pose2D__Sequence *)(untyped_member);
  hb_interfaces__msg__Pose2D__Sequence__fini(member);
  return hb_interfaces__msg__Pose2D__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_member_array[1] = {
  {
    "poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hb_interfaces__msg__Poses2D, poses),  // bytes offset in struct
    NULL,  // default value
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__size_function__Poses2D__poses,  // size() function pointer
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__get_const_function__Poses2D__poses,  // get_const(index) function pointer
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__get_function__Poses2D__poses,  // get(index) function pointer
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__fetch_function__Poses2D__poses,  // fetch(index, &value) function pointer
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__assign_function__Poses2D__poses,  // assign(index, value) function pointer
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__resize_function__Poses2D__poses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_members = {
  "hb_interfaces__msg",  // message namespace
  "Poses2D",  // message name
  1,  // number of fields
  sizeof(hb_interfaces__msg__Poses2D),
  hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_member_array,  // message members
  hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_init_function,  // function to initialize message memory (memory has to be allocated)
  hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_type_support_handle = {
  0,
  &hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hb_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hb_interfaces, msg, Poses2D)() {
  hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hb_interfaces, msg, Pose2D)();
  if (!hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_type_support_handle.typesupport_identifier) {
    hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hb_interfaces__msg__Poses2D__rosidl_typesupport_introspection_c__Poses2D_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
