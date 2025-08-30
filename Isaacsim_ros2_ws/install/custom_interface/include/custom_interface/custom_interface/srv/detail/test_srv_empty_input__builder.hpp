// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:srv/TestSrvEmptyInput.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__BUILDER_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/srv/detail/test_srv_empty_input__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::TestSrvEmptyInput_Request>()
{
  return ::custom_interface::srv::TestSrvEmptyInput_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_interface


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_TestSrvEmptyInput_Response_result
{
public:
  Init_TestSrvEmptyInput_Response_result()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::srv::TestSrvEmptyInput_Response result(::custom_interface::srv::TestSrvEmptyInput_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::TestSrvEmptyInput_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::TestSrvEmptyInput_Response>()
{
  return custom_interface::srv::builder::Init_TestSrvEmptyInput_Response_result();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__BUILDER_HPP_
