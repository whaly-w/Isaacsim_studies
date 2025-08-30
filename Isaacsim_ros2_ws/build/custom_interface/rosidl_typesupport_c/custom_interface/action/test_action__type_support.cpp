// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from custom_interface:action/TestAction.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "custom_interface/action/detail/test_action__struct.h"
#include "custom_interface/action/detail/test_action__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_Goal_type_support_ids_t;

static const _TestAction_Goal_type_support_ids_t _TestAction_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_Goal_type_support_symbol_names_t _TestAction_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_Goal)),
  }
};

typedef struct _TestAction_Goal_type_support_data_t
{
  void * data[2];
} _TestAction_Goal_type_support_data_t;

static _TestAction_Goal_type_support_data_t _TestAction_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_Goal_message_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_TestAction_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_TestAction_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestAction_Goal_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_Goal_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_Goal)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_Result_type_support_ids_t;

static const _TestAction_Result_type_support_ids_t _TestAction_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_Result_type_support_symbol_names_t _TestAction_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_Result)),
  }
};

typedef struct _TestAction_Result_type_support_data_t
{
  void * data[2];
} _TestAction_Result_type_support_data_t;

static _TestAction_Result_type_support_data_t _TestAction_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_Result_message_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_Result_message_typesupport_ids.typesupport_identifier[0],
  &_TestAction_Result_message_typesupport_symbol_names.symbol_name[0],
  &_TestAction_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestAction_Result_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_Result_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_Result)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_Feedback_type_support_ids_t;

static const _TestAction_Feedback_type_support_ids_t _TestAction_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_Feedback_type_support_symbol_names_t _TestAction_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_Feedback)),
  }
};

typedef struct _TestAction_Feedback_type_support_data_t
{
  void * data[2];
} _TestAction_Feedback_type_support_data_t;

static _TestAction_Feedback_type_support_data_t _TestAction_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_Feedback_message_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_TestAction_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_TestAction_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestAction_Feedback_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_Feedback_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_Feedback)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_SendGoal_Request_type_support_ids_t;

static const _TestAction_SendGoal_Request_type_support_ids_t _TestAction_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_SendGoal_Request_type_support_symbol_names_t _TestAction_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_SendGoal_Request)),
  }
};

typedef struct _TestAction_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _TestAction_SendGoal_Request_type_support_data_t;

static _TestAction_SendGoal_Request_type_support_data_t _TestAction_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_SendGoal_Request_message_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_TestAction_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_TestAction_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestAction_SendGoal_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_SendGoal_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_SendGoal_Request)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_SendGoal_Response_type_support_ids_t;

static const _TestAction_SendGoal_Response_type_support_ids_t _TestAction_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_SendGoal_Response_type_support_symbol_names_t _TestAction_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_SendGoal_Response)),
  }
};

typedef struct _TestAction_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _TestAction_SendGoal_Response_type_support_data_t;

static _TestAction_SendGoal_Response_type_support_data_t _TestAction_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_SendGoal_Response_message_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_TestAction_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_TestAction_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestAction_SendGoal_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_SendGoal_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_SendGoal_Response)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_SendGoal_type_support_ids_t;

static const _TestAction_SendGoal_type_support_ids_t _TestAction_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_SendGoal_type_support_symbol_names_t _TestAction_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_SendGoal)),
  }
};

typedef struct _TestAction_SendGoal_type_support_data_t
{
  void * data[2];
} _TestAction_SendGoal_type_support_data_t;

static _TestAction_SendGoal_type_support_data_t _TestAction_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_SendGoal_service_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_TestAction_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_TestAction_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t TestAction_SendGoal_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_SendGoal_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_SendGoal)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_SendGoal_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_GetResult_Request_type_support_ids_t;

static const _TestAction_GetResult_Request_type_support_ids_t _TestAction_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_GetResult_Request_type_support_symbol_names_t _TestAction_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_GetResult_Request)),
  }
};

typedef struct _TestAction_GetResult_Request_type_support_data_t
{
  void * data[2];
} _TestAction_GetResult_Request_type_support_data_t;

static _TestAction_GetResult_Request_type_support_data_t _TestAction_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_GetResult_Request_message_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_TestAction_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_TestAction_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestAction_GetResult_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_GetResult_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_GetResult_Request)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_GetResult_Response_type_support_ids_t;

static const _TestAction_GetResult_Response_type_support_ids_t _TestAction_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_GetResult_Response_type_support_symbol_names_t _TestAction_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_GetResult_Response)),
  }
};

typedef struct _TestAction_GetResult_Response_type_support_data_t
{
  void * data[2];
} _TestAction_GetResult_Response_type_support_data_t;

static _TestAction_GetResult_Response_type_support_data_t _TestAction_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_GetResult_Response_message_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_TestAction_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_TestAction_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestAction_GetResult_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_GetResult_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_GetResult_Response)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_GetResult_type_support_ids_t;

static const _TestAction_GetResult_type_support_ids_t _TestAction_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_GetResult_type_support_symbol_names_t _TestAction_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_GetResult)),
  }
};

typedef struct _TestAction_GetResult_type_support_data_t
{
  void * data[2];
} _TestAction_GetResult_type_support_data_t;

static _TestAction_GetResult_type_support_data_t _TestAction_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_GetResult_service_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_TestAction_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_TestAction_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t TestAction_GetResult_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_GetResult_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_GetResult)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_GetResult_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__struct.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_c
{

typedef struct _TestAction_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TestAction_FeedbackMessage_type_support_ids_t;

static const _TestAction_FeedbackMessage_type_support_ids_t _TestAction_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _TestAction_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TestAction_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TestAction_FeedbackMessage_type_support_symbol_names_t _TestAction_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, custom_interface, action, TestAction_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface, action, TestAction_FeedbackMessage)),
  }
};

typedef struct _TestAction_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _TestAction_FeedbackMessage_type_support_data_t;

static _TestAction_FeedbackMessage_type_support_data_t _TestAction_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TestAction_FeedbackMessage_message_typesupport_map = {
  2,
  "custom_interface",
  &_TestAction_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_TestAction_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_TestAction_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TestAction_FeedbackMessage_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TestAction_FeedbackMessage_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace action

}  // namespace custom_interface

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, custom_interface, action, TestAction_FeedbackMessage)() {
  return &::custom_interface::action::rosidl_typesupport_c::TestAction_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "action_msgs/msg/goal_status_array.h"
#include "action_msgs/srv/cancel_goal.h"
#include "custom_interface/action/test_action.h"
// already included above
// #include "custom_interface/action/detail/test_action__type_support.h"

static rosidl_action_type_support_t _custom_interface__action__TestAction__typesupport_c;

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(
  rosidl_typesupport_c, custom_interface, action, TestAction)()
{
  // Thread-safe by always writing the same values to the static struct
  _custom_interface__action__TestAction__typesupport_c.goal_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, custom_interface, action, TestAction_SendGoal)();
  _custom_interface__action__TestAction__typesupport_c.result_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, custom_interface, action, TestAction_GetResult)();
  _custom_interface__action__TestAction__typesupport_c.cancel_service_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, srv, CancelGoal)();
  _custom_interface__action__TestAction__typesupport_c.feedback_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, custom_interface, action, TestAction_FeedbackMessage)();
  _custom_interface__action__TestAction__typesupport_c.status_message_type_support =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c, action_msgs, msg, GoalStatusArray)();

  return &_custom_interface__action__TestAction__typesupport_c;
}

#ifdef __cplusplus
}
#endif
