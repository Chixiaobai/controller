// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/RealTimeBodyTcpCartesian.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__RealTimeBodyTcpCartesian __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__RealTimeBodyTcpCartesian __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RealTimeBodyTcpCartesian_
{
  using Type = RealTimeBodyTcpCartesian_<ContainerAllocator>;

  explicit RealTimeBodyTcpCartesian_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<double, 6>::iterator, double>(this->left_arm.begin(), this->left_arm.end(), 0.0);
      this->left_arm_valid = 0;
      std::fill<typename std::array<double, 6>::iterator, double>(this->right_arm.begin(), this->right_arm.end(), 0.0);
      this->right_arm_valid = 0;
      std::fill<typename std::array<double, 6>::iterator, double>(this->torso.begin(), this->torso.end(), 0.0);
      this->torso_valid = 0;
      this->time = 0.0;
    }
  }

  explicit RealTimeBodyTcpCartesian_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : left_arm(_alloc),
    right_arm(_alloc),
    torso(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<double, 6>::iterator, double>(this->left_arm.begin(), this->left_arm.end(), 0.0);
      this->left_arm_valid = 0;
      std::fill<typename std::array<double, 6>::iterator, double>(this->right_arm.begin(), this->right_arm.end(), 0.0);
      this->right_arm_valid = 0;
      std::fill<typename std::array<double, 6>::iterator, double>(this->torso.begin(), this->torso.end(), 0.0);
      this->torso_valid = 0;
      this->time = 0.0;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _left_arm_type =
    std::array<double, 6>;
  _left_arm_type left_arm;
  using _left_arm_valid_type =
    int8_t;
  _left_arm_valid_type left_arm_valid;
  using _right_arm_type =
    std::array<double, 6>;
  _right_arm_type right_arm;
  using _right_arm_valid_type =
    int8_t;
  _right_arm_valid_type right_arm_valid;
  using _torso_type =
    std::array<double, 6>;
  _torso_type torso;
  using _torso_valid_type =
    int8_t;
  _torso_valid_type torso_valid;
  using _time_type =
    double;
  _time_type time;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__left_arm(
    const std::array<double, 6> & _arg)
  {
    this->left_arm = _arg;
    return *this;
  }
  Type & set__left_arm_valid(
    const int8_t & _arg)
  {
    this->left_arm_valid = _arg;
    return *this;
  }
  Type & set__right_arm(
    const std::array<double, 6> & _arg)
  {
    this->right_arm = _arg;
    return *this;
  }
  Type & set__right_arm_valid(
    const int8_t & _arg)
  {
    this->right_arm_valid = _arg;
    return *this;
  }
  Type & set__torso(
    const std::array<double, 6> & _arg)
  {
    this->torso = _arg;
    return *this;
  }
  Type & set__torso_valid(
    const int8_t & _arg)
  {
    this->torso_valid = _arg;
    return *this;
  }
  Type & set__time(
    const double & _arg)
  {
    this->time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__RealTimeBodyTcpCartesian
    std::shared_ptr<controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__RealTimeBodyTcpCartesian
    std::shared_ptr<controller::msg::RealTimeBodyTcpCartesian_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RealTimeBodyTcpCartesian_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->left_arm != other.left_arm) {
      return false;
    }
    if (this->left_arm_valid != other.left_arm_valid) {
      return false;
    }
    if (this->right_arm != other.right_arm) {
      return false;
    }
    if (this->right_arm_valid != other.right_arm_valid) {
      return false;
    }
    if (this->torso != other.torso) {
      return false;
    }
    if (this->torso_valid != other.torso_valid) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    return true;
  }
  bool operator!=(const RealTimeBodyTcpCartesian_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RealTimeBodyTcpCartesian_

// alias to use template instance with default allocator
using RealTimeBodyTcpCartesian =
  controller::msg::RealTimeBodyTcpCartesian_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__STRUCT_HPP_
