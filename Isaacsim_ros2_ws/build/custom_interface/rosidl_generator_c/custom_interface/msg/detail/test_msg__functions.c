// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interface:msg/TestMsg.idl
// generated code does not contain a copyright notice
#include "custom_interface/msg/detail/test_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `test_vel`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
custom_interface__msg__TestMsg__init(custom_interface__msg__TestMsg * msg)
{
  if (!msg) {
    return false;
  }
  // test_vel
  if (!geometry_msgs__msg__Vector3__init(&msg->test_vel)) {
    custom_interface__msg__TestMsg__fini(msg);
    return false;
  }
  // test_yaw
  return true;
}

void
custom_interface__msg__TestMsg__fini(custom_interface__msg__TestMsg * msg)
{
  if (!msg) {
    return;
  }
  // test_vel
  geometry_msgs__msg__Vector3__fini(&msg->test_vel);
  // test_yaw
}

bool
custom_interface__msg__TestMsg__are_equal(const custom_interface__msg__TestMsg * lhs, const custom_interface__msg__TestMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // test_vel
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->test_vel), &(rhs->test_vel)))
  {
    return false;
  }
  // test_yaw
  if (lhs->test_yaw != rhs->test_yaw) {
    return false;
  }
  return true;
}

bool
custom_interface__msg__TestMsg__copy(
  const custom_interface__msg__TestMsg * input,
  custom_interface__msg__TestMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // test_vel
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->test_vel), &(output->test_vel)))
  {
    return false;
  }
  // test_yaw
  output->test_yaw = input->test_yaw;
  return true;
}

custom_interface__msg__TestMsg *
custom_interface__msg__TestMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__TestMsg * msg = (custom_interface__msg__TestMsg *)allocator.allocate(sizeof(custom_interface__msg__TestMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface__msg__TestMsg));
  bool success = custom_interface__msg__TestMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface__msg__TestMsg__destroy(custom_interface__msg__TestMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface__msg__TestMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface__msg__TestMsg__Sequence__init(custom_interface__msg__TestMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__TestMsg * data = NULL;

  if (size) {
    data = (custom_interface__msg__TestMsg *)allocator.zero_allocate(size, sizeof(custom_interface__msg__TestMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface__msg__TestMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface__msg__TestMsg__fini(&data[i - 1]);
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
custom_interface__msg__TestMsg__Sequence__fini(custom_interface__msg__TestMsg__Sequence * array)
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
      custom_interface__msg__TestMsg__fini(&array->data[i]);
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

custom_interface__msg__TestMsg__Sequence *
custom_interface__msg__TestMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface__msg__TestMsg__Sequence * array = (custom_interface__msg__TestMsg__Sequence *)allocator.allocate(sizeof(custom_interface__msg__TestMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface__msg__TestMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface__msg__TestMsg__Sequence__destroy(custom_interface__msg__TestMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface__msg__TestMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface__msg__TestMsg__Sequence__are_equal(const custom_interface__msg__TestMsg__Sequence * lhs, const custom_interface__msg__TestMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface__msg__TestMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface__msg__TestMsg__Sequence__copy(
  const custom_interface__msg__TestMsg__Sequence * input,
  custom_interface__msg__TestMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface__msg__TestMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interface__msg__TestMsg * data =
      (custom_interface__msg__TestMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface__msg__TestMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interface__msg__TestMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interface__msg__TestMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
