// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/CstmInforobot.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/cstm_inforobot__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `room_name`
#include "rosidl_runtime_c/string_functions.h"

bool
interfaces__msg__CstmInforobot__init(interfaces__msg__CstmInforobot * msg)
{
  if (!msg) {
    return false;
  }
  // room_id
  // room_name
  if (!rosidl_runtime_c__String__init(&msg->room_name)) {
    interfaces__msg__CstmInforobot__fini(msg);
    return false;
  }
  // battery_level
  return true;
}

void
interfaces__msg__CstmInforobot__fini(interfaces__msg__CstmInforobot * msg)
{
  if (!msg) {
    return;
  }
  // room_id
  // room_name
  rosidl_runtime_c__String__fini(&msg->room_name);
  // battery_level
}

bool
interfaces__msg__CstmInforobot__are_equal(const interfaces__msg__CstmInforobot * lhs, const interfaces__msg__CstmInforobot * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // room_id
  if (lhs->room_id != rhs->room_id) {
    return false;
  }
  // room_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->room_name), &(rhs->room_name)))
  {
    return false;
  }
  // battery_level
  if (lhs->battery_level != rhs->battery_level) {
    return false;
  }
  return true;
}

bool
interfaces__msg__CstmInforobot__copy(
  const interfaces__msg__CstmInforobot * input,
  interfaces__msg__CstmInforobot * output)
{
  if (!input || !output) {
    return false;
  }
  // room_id
  output->room_id = input->room_id;
  // room_name
  if (!rosidl_runtime_c__String__copy(
      &(input->room_name), &(output->room_name)))
  {
    return false;
  }
  // battery_level
  output->battery_level = input->battery_level;
  return true;
}

interfaces__msg__CstmInforobot *
interfaces__msg__CstmInforobot__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__CstmInforobot * msg = (interfaces__msg__CstmInforobot *)allocator.allocate(sizeof(interfaces__msg__CstmInforobot), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__CstmInforobot));
  bool success = interfaces__msg__CstmInforobot__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__CstmInforobot__destroy(interfaces__msg__CstmInforobot * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__CstmInforobot__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__CstmInforobot__Sequence__init(interfaces__msg__CstmInforobot__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__CstmInforobot * data = NULL;

  if (size) {
    data = (interfaces__msg__CstmInforobot *)allocator.zero_allocate(size, sizeof(interfaces__msg__CstmInforobot), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__CstmInforobot__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__CstmInforobot__fini(&data[i - 1]);
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
interfaces__msg__CstmInforobot__Sequence__fini(interfaces__msg__CstmInforobot__Sequence * array)
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
      interfaces__msg__CstmInforobot__fini(&array->data[i]);
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

interfaces__msg__CstmInforobot__Sequence *
interfaces__msg__CstmInforobot__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__CstmInforobot__Sequence * array = (interfaces__msg__CstmInforobot__Sequence *)allocator.allocate(sizeof(interfaces__msg__CstmInforobot__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__CstmInforobot__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__CstmInforobot__Sequence__destroy(interfaces__msg__CstmInforobot__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__CstmInforobot__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__CstmInforobot__Sequence__are_equal(const interfaces__msg__CstmInforobot__Sequence * lhs, const interfaces__msg__CstmInforobot__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__CstmInforobot__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__CstmInforobot__Sequence__copy(
  const interfaces__msg__CstmInforobot__Sequence * input,
  interfaces__msg__CstmInforobot__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__CstmInforobot);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__CstmInforobot * data =
      (interfaces__msg__CstmInforobot *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__CstmInforobot__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__CstmInforobot__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__CstmInforobot__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
