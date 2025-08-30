// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from custom_interface:action/TurtleDrawAction.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_Goal_type_support_ids_t;

static const _TurtleDrawAction_Goal_type_support_ids_t _TurtleDrawAction_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_Goal_type_support_symbol_names_t _TurtleDrawAction_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_Goal)),
  }
};

typedef struct _TurtleDrawAction_Goal_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_Goal_type_support_data_t;

static _TurtleDrawAction_Goal_type_support_data_t _TurtleDrawAction_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_Goal_message_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TurtleDrawAction_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface::action::TurtleDrawAction_Goal>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_Goal)() {
  return get_message_type_support_handle<custom_interface::action::TurtleDrawAction_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_Result_type_support_ids_t;

static const _TurtleDrawAction_Result_type_support_ids_t _TurtleDrawAction_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_Result_type_support_symbol_names_t _TurtleDrawAction_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_Result)),
  }
};

typedef struct _TurtleDrawAction_Result_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_Result_type_support_data_t;

static _TurtleDrawAction_Result_type_support_data_t _TurtleDrawAction_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_Result_message_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_Result_message_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_Result_message_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TurtleDrawAction_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface::action::TurtleDrawAction_Result>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_Result)() {
  return get_message_type_support_handle<custom_interface::action::TurtleDrawAction_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_Feedback_type_support_ids_t;

static const _TurtleDrawAction_Feedback_type_support_ids_t _TurtleDrawAction_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_Feedback_type_support_symbol_names_t _TurtleDrawAction_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_Feedback)),
  }
};

typedef struct _TurtleDrawAction_Feedback_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_Feedback_type_support_data_t;

static _TurtleDrawAction_Feedback_type_support_data_t _TurtleDrawAction_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_Feedback_message_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TurtleDrawAction_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface::action::TurtleDrawAction_Feedback>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_Feedback)() {
  return get_message_type_support_handle<custom_interface::action::TurtleDrawAction_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_SendGoal_Request_type_support_ids_t;

static const _TurtleDrawAction_SendGoal_Request_type_support_ids_t _TurtleDrawAction_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_SendGoal_Request_type_support_symbol_names_t _TurtleDrawAction_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_SendGoal_Request)),
  }
};

typedef struct _TurtleDrawAction_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_SendGoal_Request_type_support_data_t;

static _TurtleDrawAction_SendGoal_Request_type_support_data_t _TurtleDrawAction_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_SendGoal_Request_message_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TurtleDrawAction_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface::action::TurtleDrawAction_SendGoal_Request>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_SendGoal_Request)() {
  return get_message_type_support_handle<custom_interface::action::TurtleDrawAction_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_SendGoal_Response_type_support_ids_t;

static const _TurtleDrawAction_SendGoal_Response_type_support_ids_t _TurtleDrawAction_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_SendGoal_Response_type_support_symbol_names_t _TurtleDrawAction_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_SendGoal_Response)),
  }
};

typedef struct _TurtleDrawAction_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_SendGoal_Response_type_support_data_t;

static _TurtleDrawAction_SendGoal_Response_type_support_data_t _TurtleDrawAction_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_SendGoal_Response_message_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TurtleDrawAction_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface::action::TurtleDrawAction_SendGoal_Response>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_SendGoal_Response)() {
  return get_message_type_support_handle<custom_interface::action::TurtleDrawAction_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_SendGoal_type_support_ids_t;

static const _TurtleDrawAction_SendGoal_type_support_ids_t _TurtleDrawAction_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_SendGoal_type_support_symbol_names_t _TurtleDrawAction_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_SendGoal)),
  }
};

typedef struct _TurtleDrawAction_SendGoal_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_SendGoal_type_support_data_t;

static _TurtleDrawAction_SendGoal_type_support_data_t _TurtleDrawAction_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_SendGoal_service_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t TurtleDrawAction_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<custom_interface::action::TurtleDrawAction_SendGoal>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_SendGoal)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<custom_interface::action::TurtleDrawAction_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_GetResult_Request_type_support_ids_t;

static const _TurtleDrawAction_GetResult_Request_type_support_ids_t _TurtleDrawAction_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_GetResult_Request_type_support_symbol_names_t _TurtleDrawAction_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_GetResult_Request)),
  }
};

typedef struct _TurtleDrawAction_GetResult_Request_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_GetResult_Request_type_support_data_t;

static _TurtleDrawAction_GetResult_Request_type_support_data_t _TurtleDrawAction_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_GetResult_Request_message_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TurtleDrawAction_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface::action::TurtleDrawAction_GetResult_Request>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_GetResult_Request)() {
  return get_message_type_support_handle<custom_interface::action::TurtleDrawAction_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_GetResult_Response_type_support_ids_t;

static const _TurtleDrawAction_GetResult_Response_type_support_ids_t _TurtleDrawAction_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_GetResult_Response_type_support_symbol_names_t _TurtleDrawAction_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_GetResult_Response)),
  }
};

typedef struct _TurtleDrawAction_GetResult_Response_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_GetResult_Response_type_support_data_t;

static _TurtleDrawAction_GetResult_Response_type_support_data_t _TurtleDrawAction_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_GetResult_Response_message_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TurtleDrawAction_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface::action::TurtleDrawAction_GetResult_Response>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_GetResult_Response)() {
  return get_message_type_support_handle<custom_interface::action::TurtleDrawAction_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_GetResult_type_support_ids_t;

static const _TurtleDrawAction_GetResult_type_support_ids_t _TurtleDrawAction_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_GetResult_type_support_symbol_names_t _TurtleDrawAction_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_GetResult)),
  }
};

typedef struct _TurtleDrawAction_GetResult_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_GetResult_type_support_data_t;

static _TurtleDrawAction_GetResult_type_support_data_t _TurtleDrawAction_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_GetResult_service_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t TurtleDrawAction_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<custom_interface::action::TurtleDrawAction_GetResult>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_GetResult)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<custom_interface::action::TurtleDrawAction_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _TurtleDrawAction_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _TurtleDrawAction_FeedbackMessage_type_support_ids_t;

static const _TurtleDrawAction_FeedbackMessage_type_support_ids_t _TurtleDrawAction_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _TurtleDrawAction_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _TurtleDrawAction_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _TurtleDrawAction_FeedbackMessage_type_support_symbol_names_t _TurtleDrawAction_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface, action, TurtleDrawAction_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, custom_interface, action, TurtleDrawAction_FeedbackMessage)),
  }
};

typedef struct _TurtleDrawAction_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _TurtleDrawAction_FeedbackMessage_type_support_data_t;

static _TurtleDrawAction_FeedbackMessage_type_support_data_t _TurtleDrawAction_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _TurtleDrawAction_FeedbackMessage_message_typesupport_map = {
  2,
  "custom_interface",
  &_TurtleDrawAction_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_TurtleDrawAction_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_TurtleDrawAction_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t TurtleDrawAction_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_TurtleDrawAction_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<custom_interface::action::TurtleDrawAction_FeedbackMessage>()
{
  return &::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction_FeedbackMessage)() {
  return get_message_type_support_handle<custom_interface::action::TurtleDrawAction_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "custom_interface/action/detail/turtle_draw_action__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace custom_interface
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t TurtleDrawAction_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace custom_interface

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<custom_interface::action::TurtleDrawAction>()
{
  using ::custom_interface::action::rosidl_typesupport_cpp::TurtleDrawAction_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  TurtleDrawAction_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::custom_interface::action::TurtleDrawAction::Impl::SendGoalService>();
  TurtleDrawAction_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::custom_interface::action::TurtleDrawAction::Impl::GetResultService>();
  TurtleDrawAction_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::custom_interface::action::TurtleDrawAction::Impl::CancelGoalService>();
  TurtleDrawAction_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::custom_interface::action::TurtleDrawAction::Impl::FeedbackMessage>();
  TurtleDrawAction_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::custom_interface::action::TurtleDrawAction::Impl::GoalStatusMessage>();
  return &TurtleDrawAction_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__ACTION_SYMBOL_NAME(rosidl_typesupport_cpp, custom_interface, action, TurtleDrawAction)() {
  return ::rosidl_typesupport_cpp::get_action_type_support_handle<custom_interface::action::TurtleDrawAction>();
}

#ifdef __cplusplus
}
#endif
