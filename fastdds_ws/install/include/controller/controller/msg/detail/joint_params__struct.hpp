// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/JointParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__JOINT_PARAMS__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__JOINT_PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__JointParams __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__JointParams __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointParams_
{
  using Type = JointParams_<ContainerAllocator>;

  explicit JointParams_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_index = 0ul;
      this->max_pos = 0.0;
      this->min_pos = 0.0;
      this->max_vel = 0.0;
      this->max_acc = 0.0;
    }
  }

  explicit JointParams_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_index = 0ul;
      this->max_pos = 0.0;
      this->min_pos = 0.0;
      this->max_vel = 0.0;
      this->max_acc = 0.0;
    }
  }

  // field types and members
  using _joint_index_type =
    uint32_t;
  _joint_index_type joint_index;
  using _max_pos_type =
    double;
  _max_pos_type max_pos;
  using _min_pos_type =
    double;
  _min_pos_type min_pos;
  using _max_vel_type =
    double;
  _max_vel_type max_vel;
  using _max_acc_type =
    double;
  _max_acc_type max_acc;

  // setters for named parameter idiom
  Type & set__joint_index(
    const uint32_t & _arg)
  {
    this->joint_index = _arg;
    return *this;
  }
  Type & set__max_pos(
    const double & _arg)
  {
    this->max_pos = _arg;
    return *this;
  }
  Type & set__min_pos(
    const double & _arg)
  {
    this->min_pos = _arg;
    return *this;
  }
  Type & set__max_vel(
    const double & _arg)
  {
    this->max_vel = _arg;
    return *this;
  }
  Type & set__max_acc(
    const double & _arg)
  {
    this->max_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::JointParams_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::JointParams_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::JointParams_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::JointParams_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::JointParams_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::JointParams_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::JointParams_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::JointParams_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::JointParams_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::JointParams_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__JointParams
    std::shared_ptr<controller::msg::JointParams_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__JointParams
    std::shared_ptr<controller::msg::JointParams_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointParams_ & other) const
  {
    if (this->joint_index != other.joint_index) {
      return false;
    }
    if (this->max_pos != other.max_pos) {
      return false;
    }
    if (this->min_pos != other.min_pos) {
      return false;
    }
    if (this->max_vel != other.max_vel) {
      return false;
    }
    if (this->max_acc != other.max_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointParams_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointParams_

// alias to use template instance with default allocator
using JointParams =
  controller::msg::JointParams_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__JOINT_PARAMS__STRUCT_HPP_
