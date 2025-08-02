// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:srv/TestSrv.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV__BUILDER_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/srv/detail/test_srv__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_TestSrv_Request_b
{
public:
  explicit Init_TestSrv_Request_b(::custom_interface::srv::TestSrv_Request & msg)
  : msg_(msg)
  {}
  ::custom_interface::srv::TestSrv_Request b(::custom_interface::srv::TestSrv_Request::_b_type arg)
  {
    msg_.b = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::TestSrv_Request msg_;
};

class Init_TestSrv_Request_a
{
public:
  Init_TestSrv_Request_a()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TestSrv_Request_b a(::custom_interface::srv::TestSrv_Request::_a_type arg)
  {
    msg_.a = std::move(arg);
    return Init_TestSrv_Request_b(msg_);
  }

private:
  ::custom_interface::srv::TestSrv_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::TestSrv_Request>()
{
  return custom_interface::srv::builder::Init_TestSrv_Request_a();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_TestSrv_Response_sum
{
public:
  Init_TestSrv_Response_sum()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::srv::TestSrv_Response sum(::custom_interface::srv::TestSrv_Response::_sum_type arg)
  {
    msg_.sum = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::TestSrv_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::TestSrv_Response>()
{
  return custom_interface::srv::builder::Init_TestSrv_Response_sum();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV__BUILDER_HPP_
