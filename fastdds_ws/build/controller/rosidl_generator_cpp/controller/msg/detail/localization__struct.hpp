// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/Localization.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__LOCALIZATION__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__LOCALIZATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__msg__Localization __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__Localization __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Localization_
{
  using Type = Localization_<ContainerAllocator>;

  explicit Localization_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confidence = 0.0;
      this->valid_flag = 0;
      this->error_msgs = 0l;
      this->trajectory_id = 0l;
    }
  }

  explicit Localization_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->confidence = 0.0;
      this->valid_flag = 0;
      this->error_msgs = 0l;
      this->trajectory_id = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _confidence_type =
    double;
  _confidence_type confidence;
  using _valid_flag_type =
    int8_t;
  _valid_flag_type valid_flag;
  using _error_msgs_type =
    int32_t;
  _error_msgs_type error_msgs;
  using _trajectory_id_type =
    int32_t;
  _trajectory_id_type trajectory_id;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__confidence(
    const double & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__valid_flag(
    const int8_t & _arg)
  {
    this->valid_flag = _arg;
    return *this;
  }
  Type & set__error_msgs(
    const int32_t & _arg)
  {
    this->error_msgs = _arg;
    return *this;
  }
  Type & set__trajectory_id(
    const int32_t & _arg)
  {
    this->trajectory_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::Localization_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::Localization_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::Localization_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::Localization_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::Localization_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::Localization_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::Localization_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::Localization_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::Localization_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::Localization_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__Localization
    std::shared_ptr<controller::msg::Localization_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__Localization
    std::shared_ptr<controller::msg::Localization_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Localization_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->valid_flag != other.valid_flag) {
      return false;
    }
    if (this->error_msgs != other.error_msgs) {
      return false;
    }
    if (this->trajectory_id != other.trajectory_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const Localization_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Localization_

// alias to use template instance with default allocator
using Localization =
  controller::msg::Localization_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__LOCALIZATION__STRUCT_HPP_
