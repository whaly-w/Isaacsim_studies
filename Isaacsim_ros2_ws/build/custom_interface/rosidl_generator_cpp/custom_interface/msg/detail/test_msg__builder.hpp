// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:msg/TestMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__BUILDER_HPP_
#define CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/msg/detail/test_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace msg
{

namespace builder
{

class Init_TestMsg_test_yaw
{
public:
  explicit Init_TestMsg_test_yaw(::custom_interface::msg::TestMsg & msg)
  : msg_(msg)
  {}
  ::custom_interface::msg::TestMsg test_yaw(::custom_interface::msg::TestMsg::_test_yaw_type arg)
  {
    msg_.test_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::msg::TestMsg msg_;
};

class Init_TestMsg_test_vel
{
public:
  Init_TestMsg_test_vel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TestMsg_test_yaw test_vel(::custom_interface::msg::TestMsg::_test_vel_type arg)
  {
    msg_.test_vel = std::move(arg);
    return Init_TestMsg_test_yaw(msg_);
  }

private:
  ::custom_interface::msg::TestMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::msg::TestMsg>()
{
  return custom_interface::msg::builder::Init_TestMsg_test_vel();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__MSG__DETAIL__TEST_MSG__BUILDER_HPP_
