// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:srv/TestSrvEmptyInput.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__STRUCT_H_
#define CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/TestSrvEmptyInput in the package custom_interface.
typedef struct custom_interface__srv__TestSrvEmptyInput_Request
{
  uint8_t structure_needs_at_least_one_member;
} custom_interface__srv__TestSrvEmptyInput_Request;

// Struct for a sequence of custom_interface__srv__TestSrvEmptyInput_Request.
typedef struct custom_interface__srv__TestSrvEmptyInput_Request__Sequence
{
  custom_interface__srv__TestSrvEmptyInput_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__TestSrvEmptyInput_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/TestSrvEmptyInput in the package custom_interface.
typedef struct custom_interface__srv__TestSrvEmptyInput_Response
{
  bool result;
} custom_interface__srv__TestSrvEmptyInput_Response;

// Struct for a sequence of custom_interface__srv__TestSrvEmptyInput_Response.
typedef struct custom_interface__srv__TestSrvEmptyInput_Response__Sequence
{
  custom_interface__srv__TestSrvEmptyInput_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__TestSrvEmptyInput_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__STRUCT_H_
