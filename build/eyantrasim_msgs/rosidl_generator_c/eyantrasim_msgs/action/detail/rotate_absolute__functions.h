// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from eyantrasim_msgs:action/RotateAbsolute.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__ACTION__DETAIL__ROTATE_ABSOLUTE__FUNCTIONS_H_
#define EYANTRASIM_MSGS__ACTION__DETAIL__ROTATE_ABSOLUTE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "eyantrasim_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "eyantrasim_msgs/action/detail/rotate_absolute__struct.h"

/// Initialize action/RotateAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__action__RotateAbsolute_Goal
 * )) before or use
 * eyantrasim_msgs__action__RotateAbsolute_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Goal__init(eyantrasim_msgs__action__RotateAbsolute_Goal * msg);

/// Finalize action/RotateAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Goal__fini(eyantrasim_msgs__action__RotateAbsolute_Goal * msg);

/// Create action/RotateAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__action__RotateAbsolute_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_Goal *
eyantrasim_msgs__action__RotateAbsolute_Goal__create();

/// Destroy action/RotateAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Goal__destroy(eyantrasim_msgs__action__RotateAbsolute_Goal * msg);

/// Check for action/RotateAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Goal__are_equal(const eyantrasim_msgs__action__RotateAbsolute_Goal * lhs, const eyantrasim_msgs__action__RotateAbsolute_Goal * rhs);

/// Copy a action/RotateAbsolute message.
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
eyantrasim_msgs__action__RotateAbsolute_Goal__copy(
  const eyantrasim_msgs__action__RotateAbsolute_Goal * input,
  eyantrasim_msgs__action__RotateAbsolute_Goal * output);

/// Initialize array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__action__RotateAbsolute_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence__init(eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence * array, size_t size);

/// Finalize array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence__fini(eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence * array);

/// Create array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence *
eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence__create(size_t size);

/// Destroy array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence__destroy(eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence * array);

/// Check for action/RotateAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence__are_equal(const eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence * lhs, const eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence * rhs);

/// Copy an array of action/RotateAbsolute messages.
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
eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence__copy(
  const eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence * input,
  eyantrasim_msgs__action__RotateAbsolute_Goal__Sequence * output);

/// Initialize action/RotateAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__action__RotateAbsolute_Result
 * )) before or use
 * eyantrasim_msgs__action__RotateAbsolute_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Result__init(eyantrasim_msgs__action__RotateAbsolute_Result * msg);

/// Finalize action/RotateAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Result__fini(eyantrasim_msgs__action__RotateAbsolute_Result * msg);

/// Create action/RotateAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__action__RotateAbsolute_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_Result *
eyantrasim_msgs__action__RotateAbsolute_Result__create();

/// Destroy action/RotateAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Result__destroy(eyantrasim_msgs__action__RotateAbsolute_Result * msg);

/// Check for action/RotateAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Result__are_equal(const eyantrasim_msgs__action__RotateAbsolute_Result * lhs, const eyantrasim_msgs__action__RotateAbsolute_Result * rhs);

/// Copy a action/RotateAbsolute message.
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
eyantrasim_msgs__action__RotateAbsolute_Result__copy(
  const eyantrasim_msgs__action__RotateAbsolute_Result * input,
  eyantrasim_msgs__action__RotateAbsolute_Result * output);

/// Initialize array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__action__RotateAbsolute_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Result__Sequence__init(eyantrasim_msgs__action__RotateAbsolute_Result__Sequence * array, size_t size);

/// Finalize array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Result__Sequence__fini(eyantrasim_msgs__action__RotateAbsolute_Result__Sequence * array);

/// Create array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__action__RotateAbsolute_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_Result__Sequence *
eyantrasim_msgs__action__RotateAbsolute_Result__Sequence__create(size_t size);

/// Destroy array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Result__Sequence__destroy(eyantrasim_msgs__action__RotateAbsolute_Result__Sequence * array);

/// Check for action/RotateAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Result__Sequence__are_equal(const eyantrasim_msgs__action__RotateAbsolute_Result__Sequence * lhs, const eyantrasim_msgs__action__RotateAbsolute_Result__Sequence * rhs);

/// Copy an array of action/RotateAbsolute messages.
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
eyantrasim_msgs__action__RotateAbsolute_Result__Sequence__copy(
  const eyantrasim_msgs__action__RotateAbsolute_Result__Sequence * input,
  eyantrasim_msgs__action__RotateAbsolute_Result__Sequence * output);

/// Initialize action/RotateAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__action__RotateAbsolute_Feedback
 * )) before or use
 * eyantrasim_msgs__action__RotateAbsolute_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Feedback__init(eyantrasim_msgs__action__RotateAbsolute_Feedback * msg);

/// Finalize action/RotateAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Feedback__fini(eyantrasim_msgs__action__RotateAbsolute_Feedback * msg);

/// Create action/RotateAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__action__RotateAbsolute_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_Feedback *
eyantrasim_msgs__action__RotateAbsolute_Feedback__create();

/// Destroy action/RotateAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Feedback__destroy(eyantrasim_msgs__action__RotateAbsolute_Feedback * msg);

/// Check for action/RotateAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Feedback__are_equal(const eyantrasim_msgs__action__RotateAbsolute_Feedback * lhs, const eyantrasim_msgs__action__RotateAbsolute_Feedback * rhs);

/// Copy a action/RotateAbsolute message.
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
eyantrasim_msgs__action__RotateAbsolute_Feedback__copy(
  const eyantrasim_msgs__action__RotateAbsolute_Feedback * input,
  eyantrasim_msgs__action__RotateAbsolute_Feedback * output);

/// Initialize array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__action__RotateAbsolute_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence__init(eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence * array, size_t size);

/// Finalize array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence__fini(eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence * array);

/// Create array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence *
eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence__create(size_t size);

/// Destroy array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence__destroy(eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence * array);

/// Check for action/RotateAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence__are_equal(const eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence * lhs, const eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence * rhs);

/// Copy an array of action/RotateAbsolute messages.
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
eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence__copy(
  const eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence * input,
  eyantrasim_msgs__action__RotateAbsolute_Feedback__Sequence * output);

/// Initialize action/RotateAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request
 * )) before or use
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__init(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request * msg);

/// Finalize action/RotateAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__fini(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request * msg);

/// Create action/RotateAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request *
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__create();

/// Destroy action/RotateAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__destroy(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request * msg);

/// Check for action/RotateAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__are_equal(const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request * lhs, const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request * rhs);

/// Copy a action/RotateAbsolute message.
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
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__copy(
  const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request * input,
  eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request * output);

/// Initialize array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence__init(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence__fini(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence * array);

/// Create array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence *
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence__destroy(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence * array);

/// Check for action/RotateAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence__are_equal(const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence * lhs, const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence * rhs);

/// Copy an array of action/RotateAbsolute messages.
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
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence__copy(
  const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence * input,
  eyantrasim_msgs__action__RotateAbsolute_SendGoal_Request__Sequence * output);

/// Initialize action/RotateAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response
 * )) before or use
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__init(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response * msg);

/// Finalize action/RotateAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__fini(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response * msg);

/// Create action/RotateAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response *
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__create();

/// Destroy action/RotateAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__destroy(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response * msg);

/// Check for action/RotateAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__are_equal(const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response * lhs, const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response * rhs);

/// Copy a action/RotateAbsolute message.
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
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__copy(
  const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response * input,
  eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response * output);

/// Initialize array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence__init(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence__fini(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence * array);

/// Create array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence *
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence__destroy(eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence * array);

/// Check for action/RotateAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence__are_equal(const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence * lhs, const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence * rhs);

/// Copy an array of action/RotateAbsolute messages.
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
eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence__copy(
  const eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence * input,
  eyantrasim_msgs__action__RotateAbsolute_SendGoal_Response__Sequence * output);

/// Initialize action/RotateAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Request
 * )) before or use
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__init(eyantrasim_msgs__action__RotateAbsolute_GetResult_Request * msg);

/// Finalize action/RotateAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__fini(eyantrasim_msgs__action__RotateAbsolute_GetResult_Request * msg);

/// Create action/RotateAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request *
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__create();

/// Destroy action/RotateAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__destroy(eyantrasim_msgs__action__RotateAbsolute_GetResult_Request * msg);

/// Check for action/RotateAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__are_equal(const eyantrasim_msgs__action__RotateAbsolute_GetResult_Request * lhs, const eyantrasim_msgs__action__RotateAbsolute_GetResult_Request * rhs);

/// Copy a action/RotateAbsolute message.
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
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__copy(
  const eyantrasim_msgs__action__RotateAbsolute_GetResult_Request * input,
  eyantrasim_msgs__action__RotateAbsolute_GetResult_Request * output);

/// Initialize array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence__init(eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence__fini(eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence * array);

/// Create array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence *
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence__destroy(eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence * array);

/// Check for action/RotateAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence__are_equal(const eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence * lhs, const eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence * rhs);

/// Copy an array of action/RotateAbsolute messages.
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
eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence__copy(
  const eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence * input,
  eyantrasim_msgs__action__RotateAbsolute_GetResult_Request__Sequence * output);

/// Initialize action/RotateAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Response
 * )) before or use
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__init(eyantrasim_msgs__action__RotateAbsolute_GetResult_Response * msg);

/// Finalize action/RotateAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__fini(eyantrasim_msgs__action__RotateAbsolute_GetResult_Response * msg);

/// Create action/RotateAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response *
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__create();

/// Destroy action/RotateAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__destroy(eyantrasim_msgs__action__RotateAbsolute_GetResult_Response * msg);

/// Check for action/RotateAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__are_equal(const eyantrasim_msgs__action__RotateAbsolute_GetResult_Response * lhs, const eyantrasim_msgs__action__RotateAbsolute_GetResult_Response * rhs);

/// Copy a action/RotateAbsolute message.
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
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__copy(
  const eyantrasim_msgs__action__RotateAbsolute_GetResult_Response * input,
  eyantrasim_msgs__action__RotateAbsolute_GetResult_Response * output);

/// Initialize array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence__init(eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence__fini(eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence * array);

/// Create array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence *
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence__destroy(eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence * array);

/// Check for action/RotateAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence__are_equal(const eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence * lhs, const eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence * rhs);

/// Copy an array of action/RotateAbsolute messages.
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
eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence__copy(
  const eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence * input,
  eyantrasim_msgs__action__RotateAbsolute_GetResult_Response__Sequence * output);

/// Initialize action/RotateAbsolute message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage
 * )) before or use
 * eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__init(eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage * msg);

/// Finalize action/RotateAbsolute message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__fini(eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage * msg);

/// Create action/RotateAbsolute message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage *
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__create();

/// Destroy action/RotateAbsolute message.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__destroy(eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage * msg);

/// Check for action/RotateAbsolute message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__are_equal(const eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage * lhs, const eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage * rhs);

/// Copy a action/RotateAbsolute message.
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
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__copy(
  const eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage * input,
  eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage * output);

/// Initialize array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the number of elements and calls
 * eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence__init(eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence__fini(eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence * array);

/// Create array of action/RotateAbsolute messages.
/**
 * It allocates the memory for the array and calls
 * eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence *
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/RotateAbsolute messages.
/**
 * It calls
 * eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
void
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence__destroy(eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence * array);

/// Check for action/RotateAbsolute message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_eyantrasim_msgs
bool
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence__are_equal(const eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence * lhs, const eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence * rhs);

/// Copy an array of action/RotateAbsolute messages.
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
eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence__copy(
  const eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence * input,
  eyantrasim_msgs__action__RotateAbsolute_FeedbackMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // EYANTRASIM_MSGS__ACTION__DETAIL__ROTATE_ABSOLUTE__FUNCTIONS_H_
