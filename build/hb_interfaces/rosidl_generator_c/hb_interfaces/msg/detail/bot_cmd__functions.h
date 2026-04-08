// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hb_interfaces:msg/BotCmd.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD__FUNCTIONS_H_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hb_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "hb_interfaces/msg/detail/bot_cmd__struct.h"

/// Initialize msg/BotCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hb_interfaces__msg__BotCmd
 * )) before or use
 * hb_interfaces__msg__BotCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
bool
hb_interfaces__msg__BotCmd__init(hb_interfaces__msg__BotCmd * msg);

/// Finalize msg/BotCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
void
hb_interfaces__msg__BotCmd__fini(hb_interfaces__msg__BotCmd * msg);

/// Create msg/BotCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hb_interfaces__msg__BotCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
hb_interfaces__msg__BotCmd *
hb_interfaces__msg__BotCmd__create();

/// Destroy msg/BotCmd message.
/**
 * It calls
 * hb_interfaces__msg__BotCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
void
hb_interfaces__msg__BotCmd__destroy(hb_interfaces__msg__BotCmd * msg);

/// Check for msg/BotCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
bool
hb_interfaces__msg__BotCmd__are_equal(const hb_interfaces__msg__BotCmd * lhs, const hb_interfaces__msg__BotCmd * rhs);

/// Copy a msg/BotCmd message.
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
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
bool
hb_interfaces__msg__BotCmd__copy(
  const hb_interfaces__msg__BotCmd * input,
  hb_interfaces__msg__BotCmd * output);

/// Initialize array of msg/BotCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * hb_interfaces__msg__BotCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
bool
hb_interfaces__msg__BotCmd__Sequence__init(hb_interfaces__msg__BotCmd__Sequence * array, size_t size);

/// Finalize array of msg/BotCmd messages.
/**
 * It calls
 * hb_interfaces__msg__BotCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
void
hb_interfaces__msg__BotCmd__Sequence__fini(hb_interfaces__msg__BotCmd__Sequence * array);

/// Create array of msg/BotCmd messages.
/**
 * It allocates the memory for the array and calls
 * hb_interfaces__msg__BotCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
hb_interfaces__msg__BotCmd__Sequence *
hb_interfaces__msg__BotCmd__Sequence__create(size_t size);

/// Destroy array of msg/BotCmd messages.
/**
 * It calls
 * hb_interfaces__msg__BotCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
void
hb_interfaces__msg__BotCmd__Sequence__destroy(hb_interfaces__msg__BotCmd__Sequence * array);

/// Check for msg/BotCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
bool
hb_interfaces__msg__BotCmd__Sequence__are_equal(const hb_interfaces__msg__BotCmd__Sequence * lhs, const hb_interfaces__msg__BotCmd__Sequence * rhs);

/// Copy an array of msg/BotCmd messages.
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
ROSIDL_GENERATOR_C_PUBLIC_hb_interfaces
bool
hb_interfaces__msg__BotCmd__Sequence__copy(
  const hb_interfaces__msg__BotCmd__Sequence * input,
  hb_interfaces__msg__BotCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD__FUNCTIONS_H_
