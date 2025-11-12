// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/TcpPoseParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__TcpPoseParams __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__TcpPoseParams __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TcpPoseParams_
{
  using Type = TcpPoseParams_<ContainerAllocator>;

  explicit TcpPoseParams_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0l;
    }
  }

  explicit TcpPoseParams_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _pose_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _pose_type pose;

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

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::TcpPoseParams_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::TcpPoseParams_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::TcpPoseParams_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::TcpPoseParams_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::TcpPoseParams_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::TcpPoseParams_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::TcpPoseParams_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::TcpPoseParams_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::TcpPoseParams_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::TcpPoseParams_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__TcpPoseParams
    std::shared_ptr<controller::msg::TcpPoseParams_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__TcpPoseParams
    std::shared_ptr<controller::msg::TcpPoseParams_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TcpPoseParams_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const TcpPoseParams_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TcpPoseParams_

// alias to use template instance with default allocator
using TcpPoseParams =
  controller::msg::TcpPoseParams_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__STRUCT_HPP_
