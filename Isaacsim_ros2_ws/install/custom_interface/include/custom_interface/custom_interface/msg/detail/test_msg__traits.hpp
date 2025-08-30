// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface:msg/TestMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__TRAITS_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface/msg/detail/test_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'test_vel'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace custom_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const TestMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: test_vel
  {
    out << "test_vel: ";
    to_flow_style_yaml(msg.test_vel, out);
    out << ", ";
  }

  // member: test_yaw
  {
    out << "test_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.test_yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TestMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: test_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "test_vel:\n";
    to_block_style_yaml(msg.test_vel, out, indentation + 2);
  }

  // member: test_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "test_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.test_yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TestMsg & msg, bool use_flow_style = false)
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

}  // namespace custom_interface

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface::msg::TestMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::msg::TestMsg & msg)
{
  return custom_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::msg::TestMsg>()
{
  return "custom_interface::msg::TestMsg";
}

template<>
inline const char * name<custom_interface::msg::TestMsg>()
{
  return "custom_interface/msg/TestMsg";
}

template<>
struct has_fixed_size<custom_interface::msg::TestMsg>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct has_bounded_size<custom_interface::msg::TestMsg>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value> {};

template<>
struct is_message<custom_interface::msg::TestMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__TRAITS_HPP_
