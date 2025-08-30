// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from h1_msgs:msg/ProtoOdom.idl
// generated code does not contain a copyright notice

#ifndef H1_MSGS__MSG__DETAIL__PROTO_ODOM__STRUCT_HPP_
#define H1_MSGS__MSG__DETAIL__PROTO_ODOM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'vel'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__h1_msgs__msg__ProtoOdom __attribute__((deprecated))
#else
# define DEPRECATED__h1_msgs__msg__ProtoOdom __declspec(deprecated)
#endif

namespace h1_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ProtoOdom_
{
  using Type = ProtoOdom_<ContainerAllocator>;

  explicit ProtoOdom_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vel(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
    }
  }

  explicit ProtoOdom_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : vel(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _vel_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _vel_type vel;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__vel(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    h1_msgs::msg::ProtoOdom_<ContainerAllocator> *;
  using ConstRawPtr =
    const h1_msgs::msg::ProtoOdom_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<h1_msgs::msg::ProtoOdom_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<h1_msgs::msg::ProtoOdom_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      h1_msgs::msg::ProtoOdom_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<h1_msgs::msg::ProtoOdom_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      h1_msgs::msg::ProtoOdom_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<h1_msgs::msg::ProtoOdom_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<h1_msgs::msg::ProtoOdom_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<h1_msgs::msg::ProtoOdom_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__h1_msgs__msg__ProtoOdom
    std::shared_ptr<h1_msgs::msg::ProtoOdom_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__h1_msgs__msg__ProtoOdom
    std::shared_ptr<h1_msgs::msg::ProtoOdom_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ProtoOdom_ & other) const
  {
    if (this->vel != other.vel) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const ProtoOdom_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ProtoOdom_

// alias to use template instance with default allocator
using ProtoOdom =
  h1_msgs::msg::ProtoOdom_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace h1_msgs

#endif  // H1_MSGS__MSG__DETAIL__PROTO_ODOM__STRUCT_HPP_
