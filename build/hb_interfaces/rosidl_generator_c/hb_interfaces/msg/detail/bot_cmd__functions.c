// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hb_interfaces:msg/BotCmd.idl
// generated code does not contain a copyright notice
#include "hb_interfaces/msg/detail/bot_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hb_interfaces__msg__BotCmd__init(hb_interfaces__msg__BotCmd * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // m1
  // m2
  // m3
  // base
  // elbow
  return true;
}

void
hb_interfaces__msg__BotCmd__fini(hb_interfaces__msg__BotCmd * msg)
{
  if (!msg) {
    return;
  }
  // id
  // m1
  // m2
  // m3
  // base
  // elbow
}

bool
hb_interfaces__msg__BotCmd__are_equal(const hb_interfaces__msg__BotCmd * lhs, const hb_interfaces__msg__BotCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // m1
  if (lhs->m1 != rhs->m1) {
    return false;
  }
  // m2
  if (lhs->m2 != rhs->m2) {
    return false;
  }
  // m3
  if (lhs->m3 != rhs->m3) {
    return false;
  }
  // base
  if (lhs->base != rhs->base) {
    return false;
  }
  // elbow
  if (lhs->elbow != rhs->elbow) {
    return false;
  }
  return true;
}

bool
hb_interfaces__msg__BotCmd__copy(
  const hb_interfaces__msg__BotCmd * input,
  hb_interfaces__msg__BotCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // m1
  output->m1 = input->m1;
  // m2
  output->m2 = input->m2;
  // m3
  output->m3 = input->m3;
  // base
  output->base = input->base;
  // elbow
  output->elbow = input->elbow;
  return true;
}

hb_interfaces__msg__BotCmd *
hb_interfaces__msg__BotCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hb_interfaces__msg__BotCmd * msg = (hb_interfaces__msg__BotCmd *)allocator.allocate(sizeof(hb_interfaces__msg__BotCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hb_interfaces__msg__BotCmd));
  bool success = hb_interfaces__msg__BotCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hb_interfaces__msg__BotCmd__destroy(hb_interfaces__msg__BotCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hb_interfaces__msg__BotCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hb_interfaces__msg__BotCmd__Sequence__init(hb_interfaces__msg__BotCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hb_interfaces__msg__BotCmd * data = NULL;

  if (size) {
    data = (hb_interfaces__msg__BotCmd *)allocator.zero_allocate(size, sizeof(hb_interfaces__msg__BotCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hb_interfaces__msg__BotCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hb_interfaces__msg__BotCmd__fini(&data[i - 1]);
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
hb_interfaces__msg__BotCmd__Sequence__fini(hb_interfaces__msg__BotCmd__Sequence * array)
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
      hb_interfaces__msg__BotCmd__fini(&array->data[i]);
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

hb_interfaces__msg__BotCmd__Sequence *
hb_interfaces__msg__BotCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hb_interfaces__msg__BotCmd__Sequence * array = (hb_interfaces__msg__BotCmd__Sequence *)allocator.allocate(sizeof(hb_interfaces__msg__BotCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hb_interfaces__msg__BotCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hb_interfaces__msg__BotCmd__Sequence__destroy(hb_interfaces__msg__BotCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hb_interfaces__msg__BotCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hb_interfaces__msg__BotCmd__Sequence__are_equal(const hb_interfaces__msg__BotCmd__Sequence * lhs, const hb_interfaces__msg__BotCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hb_interfaces__msg__BotCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hb_interfaces__msg__BotCmd__Sequence__copy(
  const hb_interfaces__msg__BotCmd__Sequence * input,
  hb_interfaces__msg__BotCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hb_interfaces__msg__BotCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hb_interfaces__msg__BotCmd * data =
      (hb_interfaces__msg__BotCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hb_interfaces__msg__BotCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hb_interfaces__msg__BotCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hb_interfaces__msg__BotCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
