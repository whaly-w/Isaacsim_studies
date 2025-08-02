// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from h1_msgs:msg/ProtoOdom.idl
// generated code does not contain a copyright notice

#ifndef H1_MSGS__MSG__DETAIL__PROTO_ODOM__BUILDER_HPP_
#define H1_MSGS__MSG__DETAIL__PROTO_ODOM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "h1_msgs/msg/detail/proto_odom__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace h1_msgs
{

namespace msg
{

namespace builder
{

class Init_ProtoOdom_yaw
{
public:
  explicit Init_ProtoOdom_yaw(::h1_msgs::msg::ProtoOdom & msg)
  : msg_(msg)
  {}
  ::h1_msgs::msg::ProtoOdom yaw(::h1_msgs::msg::ProtoOdom::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::h1_msgs::msg::ProtoOdom msg_;
};

class Init_ProtoOdom_vel
{
public:
  Init_ProtoOdom_vel()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ProtoOdom_yaw vel(::h1_msgs::msg::ProtoOdom::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_ProtoOdom_yaw(msg_);
  }

private:
  ::h1_msgs::msg::ProtoOdom msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::h1_msgs::msg::ProtoOdom>()
{
  return h1_msgs::msg::builder::Init_ProtoOdom_vel();
}

}  // namespace h1_msgs

#endif  // H1_MSGS__MSG__DETAIL__PROTO_ODOM__BUILDER_HPP_
