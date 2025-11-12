// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/TcpPayloadParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__TCP_PAYLOAD_PARAMS__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__TCP_PAYLOAD_PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__TcpPayloadParams __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__TcpPayloadParams __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TcpPayloadParams_
{
  using Type = TcpPayloadParams_<ContainerAllocator>;

  explicit TcpPayloadParams_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0l;
    }
  }

  explicit TcpPayloadParams_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0l;
    }
  }

  // field types and members
  using _type_type =
    int32_t;
  _type_type type;
  using _parameters_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _parameters_type parameters;

  // setters for named parameter idiom
  Type & set__type(
    const int32_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__parameters(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->parameters = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::TcpPayloadParams_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::TcpPayloadParams_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::TcpPayloadParams_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::TcpPayloadParams_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::TcpPayloadParams_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::TcpPayloadParams_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::TcpPayloadParams_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::TcpPayloadParams_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::TcpPayloadParams_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::TcpPayloadParams_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__TcpPayloadParams
    std::shared_ptr<controller::msg::TcpPayloadParams_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__TcpPayloadParams
    std::shared_ptr<controller::msg::TcpPayloadParams_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TcpPayloadParams_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->parameters != other.parameters) {
      return false;
    }
    return true;
  }
  bool operator!=(const TcpPayloadParams_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TcpPayloadParams_

// alias to use template instance with default allocator
using TcpPayloadParams =
  controller::msg::TcpPayloadParams_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__TCP_PAYLOAD_PARAMS__STRUCT_HPP_
