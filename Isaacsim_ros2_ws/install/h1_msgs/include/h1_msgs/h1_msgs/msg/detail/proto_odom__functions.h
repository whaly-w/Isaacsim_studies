// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from h1_msgs:msg/ProtoOdom.idl
// generated code does not contain a copyright notice

#ifndef H1_MSGS__MSG__DETAIL__PROTO_ODOM__FUNCTIONS_H_
#define H1_MSGS__MSG__DETAIL__PROTO_ODOM__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "h1_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "h1_msgs/msg/detail/proto_odom__struct.h"

/// Initialize msg/ProtoOdom message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * h1_msgs__msg__ProtoOdom
 * )) before or use
 * h1_msgs__msg__ProtoOdom__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
bool
h1_msgs__msg__ProtoOdom__init(h1_msgs__msg__ProtoOdom * msg);

/// Finalize msg/ProtoOdom message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
void
h1_msgs__msg__ProtoOdom__fini(h1_msgs__msg__ProtoOdom * msg);

/// Create msg/ProtoOdom message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * h1_msgs__msg__ProtoOdom__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
h1_msgs__msg__ProtoOdom *
h1_msgs__msg__ProtoOdom__create();

/// Destroy msg/ProtoOdom message.
/**
 * It calls
 * h1_msgs__msg__ProtoOdom__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
void
h1_msgs__msg__ProtoOdom__destroy(h1_msgs__msg__ProtoOdom * msg);

/// Check for msg/ProtoOdom message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
bool
h1_msgs__msg__ProtoOdom__are_equal(const h1_msgs__msg__ProtoOdom * lhs, const h1_msgs__msg__ProtoOdom * rhs);

/// Copy a msg/ProtoOdom message.
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
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
bool
h1_msgs__msg__ProtoOdom__copy(
  const h1_msgs__msg__ProtoOdom * input,
  h1_msgs__msg__ProtoOdom * output);

/// Initialize array of msg/ProtoOdom messages.
/**
 * It allocates the memory for the number of elements and calls
 * h1_msgs__msg__ProtoOdom__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
bool
h1_msgs__msg__ProtoOdom__Sequence__init(h1_msgs__msg__ProtoOdom__Sequence * array, size_t size);

/// Finalize array of msg/ProtoOdom messages.
/**
 * It calls
 * h1_msgs__msg__ProtoOdom__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
void
h1_msgs__msg__ProtoOdom__Sequence__fini(h1_msgs__msg__ProtoOdom__Sequence * array);

/// Create array of msg/ProtoOdom messages.
/**
 * It allocates the memory for the array and calls
 * h1_msgs__msg__ProtoOdom__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
h1_msgs__msg__ProtoOdom__Sequence *
h1_msgs__msg__ProtoOdom__Sequence__create(size_t size);

/// Destroy array of msg/ProtoOdom messages.
/**
 * It calls
 * h1_msgs__msg__ProtoOdom__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
void
h1_msgs__msg__ProtoOdom__Sequence__destroy(h1_msgs__msg__ProtoOdom__Sequence * array);

/// Check for msg/ProtoOdom message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
bool
h1_msgs__msg__ProtoOdom__Sequence__are_equal(const h1_msgs__msg__ProtoOdom__Sequence * lhs, const h1_msgs__msg__ProtoOdom__Sequence * rhs);

/// Copy an array of msg/ProtoOdom messages.
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
ROSIDL_GENERATOR_C_PUBLIC_h1_msgs
bool
h1_msgs__msg__ProtoOdom__Sequence__copy(
  const h1_msgs__msg__ProtoOdom__Sequence * input,
  h1_msgs__msg__ProtoOdom__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // H1_MSGS__MSG__DETAIL__PROTO_ODOM__FUNCTIONS_H_
