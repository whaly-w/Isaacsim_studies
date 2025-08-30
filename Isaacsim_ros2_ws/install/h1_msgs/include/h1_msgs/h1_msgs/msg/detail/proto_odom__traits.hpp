// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from h1_msgs:msg/ProtoOdom.idl
// generated code does not contain a copyright notice

#ifndef H1_MSGS__MSG__DETAIL__PROTO_ODOM__TRAITS_HPP_
#define H1_MSGS__MSG__DETAIL__PROTO_ODOM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "h1_msgs/msg/detail/proto_odom__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'vel'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace h1_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ProtoOdom & msg,
  std::ostream & out)
{
  out << "{";
  // member: vel
  {
    out << "vel: ";
    to_flow_style_yaml(msg.vel, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ProtoOdom & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel:\n";
    to_block_style_yaml(msg.vel, out, indentation + 2);
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ProtoOdom & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace h1_msgs

namespace rosidl_generator_traits
{

[[deprecated("use h1_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const h1_msgs::msg::ProtoOdom & msg,
  std::ostream & out, size_t indentation = 0)
{
  h1_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use h1_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const h1_msgs::msg::ProtoOdom & msg)
{
  return h1_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<h1_msgs::msg::ProtoOdom>()
{
  return "h1_msgs::msg::ProtoOdom";
}

template<>
inline const char * name<h1_msgs::msg::ProtoOdom>()
{
  return "h1_msgs/msg/ProtoOdom";
}

template<>
struct has_fixed_size<h1_msgs::msg::ProtoOdom>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<h1_msgs::msg::ProtoOdom>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<h1_msgs::msg::ProtoOdom>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // H1_MSGS__MSG__DETAIL__PROTO_ODOM__TRAITS_HPP_
