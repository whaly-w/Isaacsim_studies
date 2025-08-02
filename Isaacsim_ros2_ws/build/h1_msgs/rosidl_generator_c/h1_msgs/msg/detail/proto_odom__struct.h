// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from h1_msgs:msg/ProtoOdom.idl
// generated code does not contain a copyright notice

#ifndef H1_MSGS__MSG__DETAIL__PROTO_ODOM__STRUCT_H_
#define H1_MSGS__MSG__DETAIL__PROTO_ODOM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'vel'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/ProtoOdom in the package h1_msgs.
typedef struct h1_msgs__msg__ProtoOdom
{
  geometry_msgs__msg__Vector3 vel;
  double yaw;
} h1_msgs__msg__ProtoOdom;

// Struct for a sequence of h1_msgs__msg__ProtoOdom.
typedef struct h1_msgs__msg__ProtoOdom__Sequence
{
  h1_msgs__msg__ProtoOdom * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} h1_msgs__msg__ProtoOdom__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // H1_MSGS__MSG__DETAIL__PROTO_ODOM__STRUCT_H_
