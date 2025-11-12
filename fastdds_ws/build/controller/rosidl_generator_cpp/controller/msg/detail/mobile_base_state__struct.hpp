// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/MobileBaseState.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__MobileBaseState __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__MobileBaseState __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MobileBaseState_
{
  using Type = MobileBaseState_<ContainerAllocator>;

  explicit MobileBaseState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
    }
  }

  explicit MobileBaseState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
    }
  }

  // field types and members
  using _linear_velocity_type =
    float;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    float;
  _angular_velocity_type angular_velocity;

  // setters for named parameter idiom
  Type & set__linear_velocity(
    const float & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const float & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::MobileBaseState_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::MobileBaseState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::MobileBaseState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::MobileBaseState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::MobileBaseState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::MobileBaseState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::MobileBaseState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::MobileBaseState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::MobileBaseState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::MobileBaseState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__MobileBaseState
    std::shared_ptr<controller::msg::MobileBaseState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__MobileBaseState
    std::shared_ptr<controller::msg::MobileBaseState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MobileBaseState_ & other) const
  {
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const MobileBaseState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MobileBaseState_

// alias to use template instance with default allocator
using MobileBaseState =
  controller::msg::MobileBaseState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__STRUCT_HPP_
