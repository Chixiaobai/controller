// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/LinearMoveParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__LinearMoveParams __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__LinearMoveParams __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LinearMoveParams_
{
  using Type = LinearMoveParams_<ContainerAllocator>;

  explicit LinearMoveParams_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0l;
      this->velocity_percent = 0.0;
      this->acceleration_percent = 0.0;
    }
  }

  explicit LinearMoveParams_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0l;
      this->velocity_percent = 0.0;
      this->acceleration_percent = 0.0;
    }
  }

  // field types and members
  using _type_type =
    int32_t;
  _type_type type;
  using _pose_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _pose_type pose;
  using _velocity_percent_type =
    double;
  _velocity_percent_type velocity_percent;
  using _acceleration_percent_type =
    double;
  _acceleration_percent_type acceleration_percent;

  // setters for named parameter idiom
  Type & set__type(
    const int32_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__pose(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__velocity_percent(
    const double & _arg)
  {
    this->velocity_percent = _arg;
    return *this;
  }
  Type & set__acceleration_percent(
    const double & _arg)
  {
    this->acceleration_percent = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::LinearMoveParams_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::LinearMoveParams_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::LinearMoveParams_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::LinearMoveParams_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::LinearMoveParams_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::LinearMoveParams_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::LinearMoveParams_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::LinearMoveParams_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::LinearMoveParams_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::LinearMoveParams_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__LinearMoveParams
    std::shared_ptr<controller::msg::LinearMoveParams_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__LinearMoveParams
    std::shared_ptr<controller::msg::LinearMoveParams_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearMoveParams_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->velocity_percent != other.velocity_percent) {
      return false;
    }
    if (this->acceleration_percent != other.acceleration_percent) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearMoveParams_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearMoveParams_

// alias to use template instance with default allocator
using LinearMoveParams =
  controller::msg::LinearMoveParams_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__STRUCT_HPP_
