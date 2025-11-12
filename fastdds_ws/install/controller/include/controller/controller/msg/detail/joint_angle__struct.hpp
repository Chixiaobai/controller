// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/JointAngle.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__JOINT_ANGLE__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__JOINT_ANGLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__JointAngle __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__JointAngle __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointAngle_
{
  using Type = JointAngle_<ContainerAllocator>;

  explicit JointAngle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_index = 0ul;
      this->target_position = 0.0;
      this->velocity = 0.0;
    }
  }

  explicit JointAngle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_index = 0ul;
      this->target_position = 0.0;
      this->velocity = 0.0;
    }
  }

  // field types and members
  using _joint_index_type =
    uint32_t;
  _joint_index_type joint_index;
  using _target_position_type =
    double;
  _target_position_type target_position;
  using _velocity_type =
    double;
  _velocity_type velocity;

  // setters for named parameter idiom
  Type & set__joint_index(
    const uint32_t & _arg)
  {
    this->joint_index = _arg;
    return *this;
  }
  Type & set__target_position(
    const double & _arg)
  {
    this->target_position = _arg;
    return *this;
  }
  Type & set__velocity(
    const double & _arg)
  {
    this->velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::JointAngle_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::JointAngle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::JointAngle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::JointAngle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::JointAngle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::JointAngle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::JointAngle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::JointAngle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::JointAngle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::JointAngle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__JointAngle
    std::shared_ptr<controller::msg::JointAngle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__JointAngle
    std::shared_ptr<controller::msg::JointAngle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointAngle_ & other) const
  {
    if (this->joint_index != other.joint_index) {
      return false;
    }
    if (this->target_position != other.target_position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointAngle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointAngle_

// alias to use template instance with default allocator
using JointAngle =
  controller::msg::JointAngle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__JOINT_ANGLE__STRUCT_HPP_
