// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/SingleJointMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__SingleJointMove_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SingleJointMove_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SingleJointMove_Request_
{
  using Type = SingleJointMove_Request_<ContainerAllocator>;

  explicit SingleJointMove_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint_index = 0ul;
      this->target_position = 0.0;
      this->velocity = 0.0;
    }
  }

  explicit SingleJointMove_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    controller::srv::SingleJointMove_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SingleJointMove_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SingleJointMove_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SingleJointMove_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SingleJointMove_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SingleJointMove_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SingleJointMove_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SingleJointMove_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SingleJointMove_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SingleJointMove_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SingleJointMove_Request
    std::shared_ptr<controller::srv::SingleJointMove_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SingleJointMove_Request
    std::shared_ptr<controller::srv::SingleJointMove_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SingleJointMove_Request_ & other) const
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
  bool operator!=(const SingleJointMove_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SingleJointMove_Request_

// alias to use template instance with default allocator
using SingleJointMove_Request =
  controller::srv::SingleJointMove_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__SingleJointMove_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SingleJointMove_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SingleJointMove_Response_
{
  using Type = SingleJointMove_Response_<ContainerAllocator>;

  explicit SingleJointMove_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->token = 0ul;
    }
  }

  explicit SingleJointMove_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->token = 0ul;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _token_type =
    uint32_t;
  _token_type token;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
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
    controller::srv::SingleJointMove_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SingleJointMove_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SingleJointMove_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SingleJointMove_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SingleJointMove_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SingleJointMove_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SingleJointMove_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SingleJointMove_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SingleJointMove_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SingleJointMove_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SingleJointMove_Response
    std::shared_ptr<controller::srv::SingleJointMove_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SingleJointMove_Response
    std::shared_ptr<controller::srv::SingleJointMove_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SingleJointMove_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->token != other.token) {
      return false;
    }
    return true;
  }
  bool operator!=(const SingleJointMove_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SingleJointMove_Response_

// alias to use template instance with default allocator
using SingleJointMove_Response =
  controller::srv::SingleJointMove_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct SingleJointMove
{
  using Request = controller::srv::SingleJointMove_Request;
  using Response = controller::srv::SingleJointMove_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__STRUCT_HPP_
