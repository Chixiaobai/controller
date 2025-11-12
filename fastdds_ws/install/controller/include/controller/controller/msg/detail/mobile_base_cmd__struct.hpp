// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/MobileBaseCmd.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__MobileBaseCmd __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__MobileBaseCmd __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MobileBaseCmd_
{
  using Type = MobileBaseCmd_<ContainerAllocator>;

  explicit MobileBaseCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
      this->lifetime = 0.0f;
    }
  }

  explicit MobileBaseCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0f;
      this->angular_velocity = 0.0f;
      this->lifetime = 0.0f;
    }
  }

  // field types and members
  using _linear_velocity_type =
    float;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    float;
  _angular_velocity_type angular_velocity;
  using _lifetime_type =
    float;
  _lifetime_type lifetime;

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
  Type & set__lifetime(
    const float & _arg)
  {
    this->lifetime = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::MobileBaseCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::MobileBaseCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::MobileBaseCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::MobileBaseCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::MobileBaseCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::MobileBaseCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::MobileBaseCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::MobileBaseCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::MobileBaseCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::MobileBaseCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__MobileBaseCmd
    std::shared_ptr<controller::msg::MobileBaseCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__MobileBaseCmd
    std::shared_ptr<controller::msg::MobileBaseCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MobileBaseCmd_ & other) const
  {
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->lifetime != other.lifetime) {
      return false;
    }
    return true;
  }
  bool operator!=(const MobileBaseCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MobileBaseCmd_

// alias to use template instance with default allocator
using MobileBaseCmd =
  controller::msg::MobileBaseCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__STRUCT_HPP_
