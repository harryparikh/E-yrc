// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eyantrasim_msgs:srv/TeleportAbsolute.idl
// generated code does not contain a copyright notice
#include "eyantrasim_msgs/srv/detail/teleport_absolute__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__init(eyantrasim_msgs__srv__TeleportAbsolute_Request * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // theta
  return true;
}

void
eyantrasim_msgs__srv__TeleportAbsolute_Request__fini(eyantrasim_msgs__srv__TeleportAbsolute_Request * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // theta
}

bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__are_equal(const eyantrasim_msgs__srv__TeleportAbsolute_Request * lhs, const eyantrasim_msgs__srv__TeleportAbsolute_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  return true;
}

bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__copy(
  const eyantrasim_msgs__srv__TeleportAbsolute_Request * input,
  eyantrasim_msgs__srv__TeleportAbsolute_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // theta
  output->theta = input->theta;
  return true;
}

eyantrasim_msgs__srv__TeleportAbsolute_Request *
eyantrasim_msgs__srv__TeleportAbsolute_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eyantrasim_msgs__srv__TeleportAbsolute_Request * msg = (eyantrasim_msgs__srv__TeleportAbsolute_Request *)allocator.allocate(sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Request));
  bool success = eyantrasim_msgs__srv__TeleportAbsolute_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
eyantrasim_msgs__srv__TeleportAbsolute_Request__destroy(eyantrasim_msgs__srv__TeleportAbsolute_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    eyantrasim_msgs__srv__TeleportAbsolute_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__init(eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eyantrasim_msgs__srv__TeleportAbsolute_Request * data = NULL;

  if (size) {
    data = (eyantrasim_msgs__srv__TeleportAbsolute_Request *)allocator.zero_allocate(size, sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eyantrasim_msgs__srv__TeleportAbsolute_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eyantrasim_msgs__srv__TeleportAbsolute_Request__fini(&data[i - 1]);
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
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__fini(eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * array)
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
      eyantrasim_msgs__srv__TeleportAbsolute_Request__fini(&array->data[i]);
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

eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence *
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * array = (eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence *)allocator.allocate(sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__destroy(eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__are_equal(const eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * lhs, const eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eyantrasim_msgs__srv__TeleportAbsolute_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence__copy(
  const eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * input,
  eyantrasim_msgs__srv__TeleportAbsolute_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    eyantrasim_msgs__srv__TeleportAbsolute_Request * data =
      (eyantrasim_msgs__srv__TeleportAbsolute_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eyantrasim_msgs__srv__TeleportAbsolute_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          eyantrasim_msgs__srv__TeleportAbsolute_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!eyantrasim_msgs__srv__TeleportAbsolute_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__init(eyantrasim_msgs__srv__TeleportAbsolute_Response * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
eyantrasim_msgs__srv__TeleportAbsolute_Response__fini(eyantrasim_msgs__srv__TeleportAbsolute_Response * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__are_equal(const eyantrasim_msgs__srv__TeleportAbsolute_Response * lhs, const eyantrasim_msgs__srv__TeleportAbsolute_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__copy(
  const eyantrasim_msgs__srv__TeleportAbsolute_Response * input,
  eyantrasim_msgs__srv__TeleportAbsolute_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

eyantrasim_msgs__srv__TeleportAbsolute_Response *
eyantrasim_msgs__srv__TeleportAbsolute_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eyantrasim_msgs__srv__TeleportAbsolute_Response * msg = (eyantrasim_msgs__srv__TeleportAbsolute_Response *)allocator.allocate(sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Response));
  bool success = eyantrasim_msgs__srv__TeleportAbsolute_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
eyantrasim_msgs__srv__TeleportAbsolute_Response__destroy(eyantrasim_msgs__srv__TeleportAbsolute_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    eyantrasim_msgs__srv__TeleportAbsolute_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__init(eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eyantrasim_msgs__srv__TeleportAbsolute_Response * data = NULL;

  if (size) {
    data = (eyantrasim_msgs__srv__TeleportAbsolute_Response *)allocator.zero_allocate(size, sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eyantrasim_msgs__srv__TeleportAbsolute_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eyantrasim_msgs__srv__TeleportAbsolute_Response__fini(&data[i - 1]);
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
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__fini(eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * array)
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
      eyantrasim_msgs__srv__TeleportAbsolute_Response__fini(&array->data[i]);
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

eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence *
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * array = (eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence *)allocator.allocate(sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__destroy(eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__are_equal(const eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * lhs, const eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eyantrasim_msgs__srv__TeleportAbsolute_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence__copy(
  const eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * input,
  eyantrasim_msgs__srv__TeleportAbsolute_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eyantrasim_msgs__srv__TeleportAbsolute_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    eyantrasim_msgs__srv__TeleportAbsolute_Response * data =
      (eyantrasim_msgs__srv__TeleportAbsolute_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eyantrasim_msgs__srv__TeleportAbsolute_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          eyantrasim_msgs__srv__TeleportAbsolute_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!eyantrasim_msgs__srv__TeleportAbsolute_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
