// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/MoveMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tcp_pose'
#include "controller/msg/detail/tcp_pose_params__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__msg__MoveMessage __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__MoveMessage __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MoveMessage_
{
  using Type = MoveMessage_<ContainerAllocator>;

  explicit MoveMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->token = 0ul;
    }
  }

  explicit MoveMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0;
      this->token = 0ul;
    }
  }

  // field types and members
  using _state_type =
    uint8_t;
  _state_type state;
  using _position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _position_type position;
  using _velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _velocity_type velocity;
  using _tcp_pose_type =
    std::vector<controller::msg::TcpPoseParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpPoseParams_<ContainerAllocator>>>;
  _tcp_pose_type tcp_pose;
  using _token_type =
    uint32_t;
  _token_type token;

  // setters for named parameter idiom
  Type & set__state(
    const uint8_t & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__tcp_pose(
    const std::vector<controller::msg::TcpPoseParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpPoseParams_<ContainerAllocator>>> & _arg)
  {
    this->tcp_pose = _arg;
    return *this;
  }
  Type & set__token(
    const uint32_t & _arg)
  {
    this->token = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::MoveMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::MoveMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::MoveMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::MoveMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::MoveMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::MoveMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::MoveMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::MoveMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::MoveMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::MoveMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__MoveMessage
    std::shared_ptr<controller::msg::MoveMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__MoveMessage
    std::shared_ptr<controller::msg::MoveMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveMessage_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->tcp_pose != other.tcp_pose) {
      return false;
    }
    if (this->token != other.token) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveMessage_

// alias to use template instance with default allocator
using MoveMessage =
  controller::msg::MoveMessage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__STRUCT_HPP_
