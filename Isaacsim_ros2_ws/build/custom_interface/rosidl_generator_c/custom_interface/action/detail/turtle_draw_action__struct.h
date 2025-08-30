// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:action/TurtleDrawAction.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__ACTION__DETAIL__TURTLE_DRAW_ACTION__STRUCT_H_
#define CUSTOM_INTERFACE__ACTION__DETAIL__TURTLE_DRAW_ACTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/TurtleDrawAction in the package custom_interface.
typedef struct custom_interface__action__TurtleDrawAction_Goal
{
  int32_t size;
} custom_interface__action__TurtleDrawAction_Goal;

// Struct for a sequence of custom_interface__action__TurtleDrawAction_Goal.
typedef struct custom_interface__action__TurtleDrawAction_Goal__Sequence
{
  custom_interface__action__TurtleDrawAction_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__action__TurtleDrawAction_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/TurtleDrawAction in the package custom_interface.
typedef struct custom_interface__action__TurtleDrawAction_Result
{
  int32_t state;
} custom_interface__action__TurtleDrawAction_Result;

// Struct for a sequence of custom_interface__action__TurtleDrawAction_Result.
typedef struct custom_interface__action__TurtleDrawAction_Result__Sequence
{
  custom_interface__action__TurtleDrawAction_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__action__TurtleDrawAction_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/TurtleDrawAction in the package custom_interface.
typedef struct custom_interface__action__TurtleDrawAction_Feedback
{
  bool is_success;
} custom_interface__action__TurtleDrawAction_Feedback;

// Struct for a sequence of custom_interface__action__TurtleDrawAction_Feedback.
typedef struct custom_interface__action__TurtleDrawAction_Feedback__Sequence
{
  custom_interface__action__TurtleDrawAction_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__action__TurtleDrawAction_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "custom_interface/action/detail/turtle_draw_action__struct.h"

/// Struct defined in action/TurtleDrawAction in the package custom_interface.
typedef struct custom_interface__action__TurtleDrawAction_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interface__action__TurtleDrawAction_Goal goal;
} custom_interface__action__TurtleDrawAction_SendGoal_Request;

// Struct for a sequence of custom_interface__action__TurtleDrawAction_SendGoal_Request.
typedef struct custom_interface__action__TurtleDrawAction_SendGoal_Request__Sequence
{
  custom_interface__action__TurtleDrawAction_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__action__TurtleDrawAction_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/TurtleDrawAction in the package custom_interface.
typedef struct custom_interface__action__TurtleDrawAction_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} custom_interface__action__TurtleDrawAction_SendGoal_Response;

// Struct for a sequence of custom_interface__action__TurtleDrawAction_SendGoal_Response.
typedef struct custom_interface__action__TurtleDrawAction_SendGoal_Response__Sequence
{
  custom_interface__action__TurtleDrawAction_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__action__TurtleDrawAction_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/TurtleDrawAction in the package custom_interface.
typedef struct custom_interface__action__TurtleDrawAction_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} custom_interface__action__TurtleDrawAction_GetResult_Request;

// Struct for a sequence of custom_interface__action__TurtleDrawAction_GetResult_Request.
typedef struct custom_interface__action__TurtleDrawAction_GetResult_Request__Sequence
{
  custom_interface__action__TurtleDrawAction_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__action__TurtleDrawAction_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.h"

/// Struct defined in action/TurtleDrawAction in the package custom_interface.
typedef struct custom_interface__action__TurtleDrawAction_GetResult_Response
{
  int8_t status;
  custom_interface__action__TurtleDrawAction_Result result;
} custom_interface__action__TurtleDrawAction_GetResult_Response;

// Struct for a sequence of custom_interface__action__TurtleDrawAction_GetResult_Response.
typedef struct custom_interface__action__TurtleDrawAction_GetResult_Response__Sequence
{
  custom_interface__action__TurtleDrawAction_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__action__TurtleDrawAction_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.h"

/// Struct defined in action/TurtleDrawAction in the package custom_interface.
typedef struct custom_interface__action__TurtleDrawAction_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  custom_interface__action__TurtleDrawAction_Feedback feedback;
} custom_interface__action__TurtleDrawAction_FeedbackMessage;

// Struct for a sequence of custom_interface__action__TurtleDrawAction_FeedbackMessage.
typedef struct custom_interface__action__TurtleDrawAction_FeedbackMessage__Sequence
{
  custom_interface__action__TurtleDrawAction_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__action__TurtleDrawAction_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__ACTION__DETAIL__TURTLE_DRAW_ACTION__STRUCT_H_
