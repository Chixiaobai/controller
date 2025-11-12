// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/OdometryMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__OdometryMessage __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__OdometryMessage __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct OdometryMessage_
{
  using Type = OdometryMessage_<ContainerAllocator>;

  explicit OdometryMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accumulated_time = 0ul;
      this->accumulated_mileage = 0ul;
    }
  }

  explicit OdometryMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accumulated_time = 0ul;
      this->accumulated_mileage = 0ul;
    }
  }

  // field types and members
  using _accumulated_time_type =
    uint32_t;
  _accumulated_time_type accumulated_time;
  using _accumulated_mileage_type =
    uint32_t;
  _accumulated_mileage_type accumulated_mileage;

  // setters for named parameter idiom
  Type & set__accumulated_time(
    const uint32_t & _arg)
  {
    this->accumulated_time = _arg;
    return *this;
  }
  Type & set__accumulated_mileage(
    const uint32_t & _arg)
  {
    this->accumulated_mileage = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::OdometryMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::OdometryMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::OdometryMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::OdometryMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::OdometryMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::OdometryMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::OdometryMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::OdometryMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::OdometryMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::OdometryMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__OdometryMessage
    std::shared_ptr<controller::msg::OdometryMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__OdometryMessage
    std::shared_ptr<controller::msg::OdometryMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const OdometryMessage_ & other) const
  {
    if (this->accumulated_time != other.accumulated_time) {
      return false;
    }
    if (this->accumulated_mileage != other.accumulated_mileage) {
      return false;
    }
    return true;
  }
  bool operator!=(const OdometryMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct OdometryMessage_

// alias to use template instance with default allocator
using OdometryMessage =
  controller::msg::OdometryMessage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__STRUCT_HPP_
