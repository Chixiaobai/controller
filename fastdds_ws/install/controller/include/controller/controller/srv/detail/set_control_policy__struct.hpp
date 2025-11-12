// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/SetControlPolicy.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CONTROL_POLICY__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CONTROL_POLICY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__SetControlPolicy_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SetControlPolicy_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetControlPolicy_Request_
{
  using Type = SetControlPolicy_Request_<ContainerAllocator>;

  explicit SetControlPolicy_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->policy = 0l;
    }
  }

  explicit SetControlPolicy_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->policy = 0l;
    }
  }

  // field types and members
  using _policy_type =
    int32_t;
  _policy_type policy;

  // setters for named parameter idiom
  Type & set__policy(
    const int32_t & _arg)
  {
    this->policy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::SetControlPolicy_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SetControlPolicy_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SetControlPolicy_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SetControlPolicy_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetControlPolicy_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetControlPolicy_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetControlPolicy_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetControlPolicy_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SetControlPolicy_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SetControlPolicy_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SetControlPolicy_Request
    std::shared_ptr<controller::srv::SetControlPolicy_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SetControlPolicy_Request
    std::shared_ptr<controller::srv::SetControlPolicy_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetControlPolicy_Request_ & other) const
  {
    if (this->policy != other.policy) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetControlPolicy_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetControlPolicy_Request_

// alias to use template instance with default allocator
using SetControlPolicy_Request =
  controller::srv::SetControlPolicy_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__SetControlPolicy_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SetControlPolicy_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetControlPolicy_Response_
{
  using Type = SetControlPolicy_Response_<ContainerAllocator>;

  explicit SetControlPolicy_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetControlPolicy_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::SetControlPolicy_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SetControlPolicy_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SetControlPolicy_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SetControlPolicy_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetControlPolicy_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetControlPolicy_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetControlPolicy_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetControlPolicy_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SetControlPolicy_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SetControlPolicy_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SetControlPolicy_Response
    std::shared_ptr<controller::srv::SetControlPolicy_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SetControlPolicy_Response
    std::shared_ptr<controller::srv::SetControlPolicy_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetControlPolicy_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetControlPolicy_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetControlPolicy_Response_

// alias to use template instance with default allocator
using SetControlPolicy_Response =
  controller::srv::SetControlPolicy_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct SetControlPolicy
{
  using Request = controller::srv::SetControlPolicy_Request;
  using Response = controller::srv::SetControlPolicy_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_CONTROL_POLICY__STRUCT_HPP_
