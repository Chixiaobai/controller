// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/EnableController.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__ENABLE_CONTROLLER__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__ENABLE_CONTROLLER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__EnableController_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__EnableController_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EnableController_Request_
{
  using Type = EnableController_Request_<ContainerAllocator>;

  explicit EnableController_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable = false;
    }
  }

  explicit EnableController_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enable = false;
    }
  }

  // field types and members
  using _enable_type =
    bool;
  _enable_type enable;

  // setters for named parameter idiom
  Type & set__enable(
    const bool & _arg)
  {
    this->enable = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::EnableController_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::EnableController_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::EnableController_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::EnableController_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::EnableController_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::EnableController_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::EnableController_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::EnableController_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::EnableController_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::EnableController_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__EnableController_Request
    std::shared_ptr<controller::srv::EnableController_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__EnableController_Request
    std::shared_ptr<controller::srv::EnableController_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnableController_Request_ & other) const
  {
    if (this->enable != other.enable) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnableController_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnableController_Request_

// alias to use template instance with default allocator
using EnableController_Request =
  controller::srv::EnableController_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__EnableController_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__EnableController_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct EnableController_Response_
{
  using Type = EnableController_Response_<ContainerAllocator>;

  explicit EnableController_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit EnableController_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    controller::srv::EnableController_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::EnableController_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::EnableController_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::EnableController_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::EnableController_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::EnableController_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::EnableController_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::EnableController_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::EnableController_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::EnableController_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__EnableController_Response
    std::shared_ptr<controller::srv::EnableController_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__EnableController_Response
    std::shared_ptr<controller::srv::EnableController_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EnableController_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const EnableController_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EnableController_Response_

// alias to use template instance with default allocator
using EnableController_Response =
  controller::srv::EnableController_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct EnableController
{
  using Request = controller::srv::EnableController_Request;
  using Response = controller::srv::EnableController_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__ENABLE_CONTROLLER__STRUCT_HPP_
