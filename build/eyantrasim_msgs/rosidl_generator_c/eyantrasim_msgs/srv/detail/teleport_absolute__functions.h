// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from eyantrasim_msgs:srv/TeleportAbsolute.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_ABSOLUTE__FUNCTIONS_H_
#define EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_ABSOLUTE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "eyantrasim_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "eyantrasim_msgs/srv/detail/teleport_absolute__struct.h"

/// Initialize srv/TeleportAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__srv__TeleportAbsolute_Request
 * )) before or use
 * eyantrasim_msgs__srv__TeleportAbsolute_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__init(eyantrasim_msgs__srv__TeleportAbsolute_Request * msg);

/// Finalize srv/TeleportAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__srv__TeleportAbsolute_Request__fini(eyantrasim_msgs__srv__TeleportAbsolute_Request * msg);

/// Create srv/TeleportAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__srv__TeleportAbsolute_Request *
eyantrasim_msgs__srv__TeleportAbsolute_Request__create();

/// Destroy srv/TeleportAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__srv__TeleportAbsolute_Request__destroy(eyantrasim_msgs__srv__TeleportAbsolute_Request * msg);

/// Check for srv/TeleportAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__are_equal(const eyantrasim_msgs__srv__TeleportAbsolute_Request * lhs, const eyantrasim_msgs__srv__TeleportAbsolute_Request * rhs);

/// Copy a srv/TeleportAbsolute message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__copy(
  const eyantrasim_msgs__srv__TeleportAbsolute_Request * input,
  eyantrasim_msgs__srv__TeleportAbsolute_Request * output);

/// Initialize array of srv/TeleportAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__init(eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * array, size_t size);

/// Finalize array of srv/TeleportAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__fini(eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * array);

/// Create array of srv/TeleportAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence *
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__create(size_t size);

/// Destroy array of srv/TeleportAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__destroy(eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * array);

/// Check for srv/TeleportAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__are_equal(const eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * lhs, const eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * rhs);

/// Copy an array of srv/TeleportAbsolute messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__copy(
  const eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * input,
  eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * output);

/// Initialize srv/TeleportAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__srv__TeleportAbsolute_Response
 * )) before or use
 * eyantrasim_msgs__srv__TeleportAbsolute_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__init(eyantrasim_msgs__srv__TeleportAbsolute_Response * msg);

/// Finalize srv/TeleportAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__srv__TeleportAbsolute_Response__fini(eyantrasim_msgs__srv__TeleportAbsolute_Response * msg);

/// Create srv/TeleportAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__srv__TeleportAbsolute_Response *
eyantrasim_msgs__srv__TeleportAbsolute_Response__create();

/// Destroy srv/TeleportAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__srv__TeleportAbsolute_Response__destroy(eyantrasim_msgs__srv__TeleportAbsolute_Response * msg);

/// Check for srv/TeleportAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__are_equal(const eyantrasim_msgs__srv__TeleportAbsolute_Response * lhs, const eyantrasim_msgs__srv__TeleportAbsolute_Response * rhs);

/// Copy a srv/TeleportAbsolute message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__copy(
  const eyantrasim_msgs__srv__TeleportAbsolute_Response * input,
  eyantrasim_msgs__srv__TeleportAbsolute_Response * output);

/// Initialize array of srv/TeleportAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__init(eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * array, size_t size);

/// Finalize array of srv/TeleportAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__fini(eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * array);

/// Create array of srv/TeleportAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence *
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__create(size_t size);

/// Destroy array of srv/TeleportAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__destroy(eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * array);

/// Check for srv/TeleportAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__are_equal(const eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * lhs, const eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * rhs);

/// Copy an array of srv/TeleportAbsolute messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__copy(
  const eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * input,
  eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__TELEPORT_ABSOLUTE__FUNCTIONS_H_
