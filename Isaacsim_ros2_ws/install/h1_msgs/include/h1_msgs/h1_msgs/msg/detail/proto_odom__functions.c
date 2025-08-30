// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from h1_msgs:msg/ProtoOdom.idl
// generated code does not contain a copyright notice
#include "h1_msgs/msg/detail/proto_odom__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `vel`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
h1_msgs__msg__ProtoOdom__init(h1_msgs__msg__ProtoOdom * msg)
{
  if (!msg) {
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Vector3__init(&msg->vel)) {
    h1_msgs__msg__ProtoOdom__fini(msg);
    return false;
  }
  // yaw
  return true;
}

void
h1_msgs__msg__ProtoOdom__fini(h1_msgs__msg__ProtoOdom * msg)
{
  if (!msg) {
    return;
  }
  // vel
  geometry_msgs__msg__Vector3__fini(&msg->vel);
  // yaw
}

bool
h1_msgs__msg__ProtoOdom__are_equal(const h1_msgs__msg__ProtoOdom * lhs, const h1_msgs__msg__ProtoOdom * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->vel), &(rhs->vel)))
  {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
h1_msgs__msg__ProtoOdom__copy(
  const h1_msgs__msg__ProtoOdom * input,
  h1_msgs__msg__ProtoOdom * output)
{
  if (!input || !output) {
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->vel), &(output->vel)))
  {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  return true;
}

h1_msgs__msg__ProtoOdom *
h1_msgs__msg__ProtoOdom__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  h1_msgs__msg__ProtoOdom * msg = (h1_msgs__msg__ProtoOdom *)allocator.allocate(sizeof(h1_msgs__msg__ProtoOdom), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(h1_msgs__msg__ProtoOdom));
  bool success = h1_msgs__msg__ProtoOdom__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
h1_msgs__msg__ProtoOdom__destroy(h1_msgs__msg__ProtoOdom * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    h1_msgs__msg__ProtoOdom__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
h1_msgs__msg__ProtoOdom__Sequence__init(h1_msgs__msg__ProtoOdom__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  h1_msgs__msg__ProtoOdom * data = NULL;

  if (size) {
    data = (h1_msgs__msg__ProtoOdom *)allocator.zero_allocate(size, sizeof(h1_msgs__msg__ProtoOdom), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = h1_msgs__msg__ProtoOdom__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        h1_msgs__msg__ProtoOdom__fini(&data[i - 1]);
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
h1_msgs__msg__ProtoOdom__Sequence__fini(h1_msgs__msg__ProtoOdom__Sequence * array)
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
      h1_msgs__msg__ProtoOdom__fini(&array->data[i]);
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

h1_msgs__msg__ProtoOdom__Sequence *
h1_msgs__msg__ProtoOdom__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  h1_msgs__msg__ProtoOdom__Sequence * array = (h1_msgs__msg__ProtoOdom__Sequence *)allocator.allocate(sizeof(h1_msgs__msg__ProtoOdom__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = h1_msgs__msg__ProtoOdom__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
h1_msgs__msg__ProtoOdom__Sequence__destroy(h1_msgs__msg__ProtoOdom__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    h1_msgs__msg__ProtoOdom__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
h1_msgs__msg__ProtoOdom__Sequence__are_equal(const h1_msgs__msg__ProtoOdom__Sequence * lhs, const h1_msgs__msg__ProtoOdom__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!h1_msgs__msg__ProtoOdom__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
h1_msgs__msg__ProtoOdom__Sequence__copy(
  const h1_msgs__msg__ProtoOdom__Sequence * input,
  h1_msgs__msg__ProtoOdom__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(h1_msgs__msg__ProtoOdom);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    h1_msgs__msg__ProtoOdom * data =
      (h1_msgs__msg__ProtoOdom *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!h1_msgs__msg__ProtoOdom__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          h1_msgs__msg__ProtoOdom__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!h1_msgs__msg__ProtoOdom__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
