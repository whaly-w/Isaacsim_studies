// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:msg/TestMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__STRUCT_H_
#define CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'test_vel'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/TestMsg in the package custom_interface.
typedef struct custom_interface__msg__TestMsg
{
  geometry_msgs__msg__Vector3 test_vel;
  double test_yaw;
} custom_interface__msg__TestMsg;

// Struct for a sequence of custom_interface__msg__TestMsg.
typedef struct custom_interface__msg__TestMsg__Sequence
{
  custom_interface__msg__TestMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__msg__TestMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__STRUCT_H_
