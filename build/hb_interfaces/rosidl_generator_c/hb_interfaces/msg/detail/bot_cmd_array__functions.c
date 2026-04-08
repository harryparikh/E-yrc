// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hb_interfaces:msg/BotCmdArray.idl
// generated code does not contain a copyright notice
#include "hb_interfaces/msg/detail/bot_cmd_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `cmds`
#include "hb_interfaces/msg/detail/bot_cmd__functions.h"

bool
hb_interfaces__msg__BotCmdArray__init(hb_interfaces__msg__BotCmdArray * msg)
{
  if (!msg) {
    return false;
  }
  // cmds
  if (!hb_interfaces__msg__BotCmd__Sequence__init(&msg->cmds, 0)) {
    hb_interfaces__msg__BotCmdArray__fini(msg);
    return false;
  }
  return true;
}

void
hb_interfaces__msg__BotCmdArray__fini(hb_interfaces__msg__BotCmdArray * msg)
{
  if (!msg) {
    return;
  }
  // cmds
  hb_interfaces__msg__BotCmd__Sequence__fini(&msg->cmds);
}

bool
hb_interfaces__msg__BotCmdArray__are_equal(const hb_interfaces__msg__BotCmdArray * lhs, const hb_interfaces__msg__BotCmdArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmds
  if (!hb_interfaces__msg__BotCmd__Sequence__are_equal(
      &(lhs->cmds), &(rhs->cmds)))
  {
    return false;
  }
  return true;
}

bool
hb_interfaces__msg__BotCmdArray__copy(
  const hb_interfaces__msg__BotCmdArray * input,
  hb_interfaces__msg__BotCmdArray * output)
{
  if (!input || !output) {
    return false;
  }
  // cmds
  if (!hb_interfaces__msg__BotCmd__Sequence__copy(
      &(input->cmds), &(output->cmds)))
  {
    return false;
  }
  return true;
}

hb_interfaces__msg__BotCmdArray *
hb_interfaces__msg__BotCmdArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hb_interfaces__msg__BotCmdArray * msg = (hb_interfaces__msg__BotCmdArray *)allocator.allocate(sizeof(hb_interfaces__msg__BotCmdArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hb_interfaces__msg__BotCmdArray));
  bool success = hb_interfaces__msg__BotCmdArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hb_interfaces__msg__BotCmdArray__destroy(hb_interfaces__msg__BotCmdArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hb_interfaces__msg__BotCmdArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hb_interfaces__msg__BotCmdArray__Sequence__init(hb_interfaces__msg__BotCmdArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hb_interfaces__msg__BotCmdArray * data = NULL;

  if (size) {
    data = (hb_interfaces__msg__BotCmdArray *)allocator.zero_allocate(size, sizeof(hb_interfaces__msg__BotCmdArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hb_interfaces__msg__BotCmdArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hb_interfaces__msg__BotCmdArray__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hb_interfaces__msg__BotCmdArray__Sequence__fini(hb_interfaces__msg__BotCmdArray__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hb_interfaces__msg__BotCmdArray__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hb_interfaces__msg__BotCmdArray__Sequence *
hb_interfaces__msg__BotCmdArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hb_interfaces__msg__BotCmdArray__Sequence * array = (hb_interfaces__msg__BotCmdArray__Sequence *)allocator.allocate(sizeof(hb_interfaces__msg__BotCmdArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hb_interfaces__msg__BotCmdArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hb_interfaces__msg__BotCmdArray__Sequence__destroy(hb_interfaces__msg__BotCmdArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hb_interfaces__msg__BotCmdArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hb_interfaces__msg__BotCmdArray__Sequence__are_equal(const hb_interfaces__msg__BotCmdArray__Sequence * lhs, const hb_interfaces__msg__BotCmdArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hb_interfaces__msg__BotCmdArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hb_interfaces__msg__BotCmdArray__Sequence__copy(
  const hb_interfaces__msg__BotCmdArray__Sequence * input,
  hb_interfaces__msg__BotCmdArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hb_interfaces__msg__BotCmdArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hb_interfaces__msg__BotCmdArray * data =
      (hb_interfaces__msg__BotCmdArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hb_interfaces__msg__BotCmdArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hb_interfaces__msg__BotCmdArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hb_interfaces__msg__BotCmdArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
