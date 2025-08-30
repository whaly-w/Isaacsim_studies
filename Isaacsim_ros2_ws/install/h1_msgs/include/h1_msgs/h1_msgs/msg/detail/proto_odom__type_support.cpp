// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from h1_msgs:msg/ProtoOdom.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "h1_msgs/msg/detail/proto_odom__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace h1_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ProtoOdom_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) h1_msgs::msg::ProtoOdom(_init);
}

void ProtoOdom_fini_function(void * message_memory)
{
  auto typed_message = static_cast<h1_msgs::msg::ProtoOdom *>(message_memory);
  typed_message->~ProtoOdom();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ProtoOdom_message_member_array[2] = {
  {
    "vel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(h1_msgs::msg::ProtoOdom, vel),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(h1_msgs::msg::ProtoOdom, yaw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ProtoOdom_message_members = {
  "h1_msgs::msg",  // message namespace
  "ProtoOdom",  // message name
  2,  // number of fields
  sizeof(h1_msgs::msg::ProtoOdom),
  ProtoOdom_message_member_array,  // message members
  ProtoOdom_init_function,  // function to initialize message memory (memory has to be allocated)
  ProtoOdom_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ProtoOdom_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ProtoOdom_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace h1_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<h1_msgs::msg::ProtoOdom>()
{
  return &::h1_msgs::msg::rosidl_typesupport_introspection_cpp::ProtoOdom_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, h1_msgs, msg, ProtoOdom)() {
  return &::h1_msgs::msg::rosidl_typesupport_introspection_cpp::ProtoOdom_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
