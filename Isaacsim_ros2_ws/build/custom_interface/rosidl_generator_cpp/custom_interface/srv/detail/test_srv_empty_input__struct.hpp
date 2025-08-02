// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interface:srv/TestSrvEmptyInput.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__STRUCT_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interface__srv__TestSrvEmptyInput_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface__srv__TestSrvEmptyInput_Request __declspec(deprecated)
#endif

namespace custom_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TestSrvEmptyInput_Request_
{
  using Type = TestSrvEmptyInput_Request_<ContainerAllocator>;

  explicit TestSrvEmptyInput_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit TestSrvEmptyInput_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface__srv__TestSrvEmptyInput_Request
    std::shared_ptr<custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface__srv__TestSrvEmptyInput_Request
    std::shared_ptr<custom_interface::srv::TestSrvEmptyInput_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TestSrvEmptyInput_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const TestSrvEmptyInput_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TestSrvEmptyInput_Request_

// alias to use template instance with default allocator
using TestSrvEmptyInput_Request =
  custom_interface::srv::TestSrvEmptyInput_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interface


#ifndef _WIN32
# define DEPRECATED__custom_interface__srv__TestSrvEmptyInput_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface__srv__TestSrvEmptyInput_Response __declspec(deprecated)
#endif

namespace custom_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct TestSrvEmptyInput_Response_
{
  using Type = TestSrvEmptyInput_Response_<ContainerAllocator>;

  explicit TestSrvEmptyInput_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  explicit TestSrvEmptyInput_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = false;
    }
  }

  // field types and members
  using _result_type =
    bool;
  _result_type result;

  // setters for named parameter idiom
  Type & set__result(
    const bool & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface__srv__TestSrvEmptyInput_Response
    std::shared_ptr<custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface__srv__TestSrvEmptyInput_Response
    std::shared_ptr<custom_interface::srv::TestSrvEmptyInput_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TestSrvEmptyInput_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const TestSrvEmptyInput_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TestSrvEmptyInput_Response_

// alias to use template instance with default allocator
using TestSrvEmptyInput_Response =
  custom_interface::srv::TestSrvEmptyInput_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interface

namespace custom_interface
{

namespace srv
{

struct TestSrvEmptyInput
{
  using Request = custom_interface::srv::TestSrvEmptyInput_Request;
  using Response = custom_interface::srv::TestSrvEmptyInput_Response;
};

}  // namespace srv

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__TEST_SRV_EMPTY_INPUT__STRUCT_HPP_
