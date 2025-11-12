// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/SetChassisMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__SetChassisMaxVel_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SetChassisMaxVel_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetChassisMaxVel_Request_
{
  using Type = SetChassisMaxVel_Request_<ContainerAllocator>;

  explicit SetChassisMaxVel_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0;
      this->angular_velocity = 0.0;
    }
  }

  explicit SetChassisMaxVel_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0;
      this->angular_velocity = 0.0;
    }
  }

  // field types and members
  using _linear_velocity_type =
    double;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    double;
  _angular_velocity_type angular_velocity;

  // setters for named parameter idiom
  Type & set__linear_velocity(
    const double & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const double & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::SetChassisMaxVel_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SetChassisMaxVel_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SetChassisMaxVel_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SetChassisMaxVel_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetChassisMaxVel_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetChassisMaxVel_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetChassisMaxVel_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetChassisMaxVel_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SetChassisMaxVel_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SetChassisMaxVel_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SetChassisMaxVel_Request
    std::shared_ptr<controller::srv::SetChassisMaxVel_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SetChassisMaxVel_Request
    std::shared_ptr<controller::srv::SetChassisMaxVel_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetChassisMaxVel_Request_ & other) const
  {
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetChassisMaxVel_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetChassisMaxVel_Request_

// alias to use template instance with default allocator
using SetChassisMaxVel_Request =
  controller::srv::SetChassisMaxVel_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__SetChassisMaxVel_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SetChassisMaxVel_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetChassisMaxVel_Response_
{
  using Type = SetChassisMaxVel_Response_<ContainerAllocator>;

  explicit SetChassisMaxVel_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetChassisMaxVel_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    controller::srv::SetChassisMaxVel_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SetChassisMaxVel_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SetChassisMaxVel_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SetChassisMaxVel_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetChassisMaxVel_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetChassisMaxVel_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetChassisMaxVel_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetChassisMaxVel_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SetChassisMaxVel_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SetChassisMaxVel_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SetChassisMaxVel_Response
    std::shared_ptr<controller::srv::SetChassisMaxVel_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SetChassisMaxVel_Response
    std::shared_ptr<controller::srv::SetChassisMaxVel_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetChassisMaxVel_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetChassisMaxVel_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetChassisMaxVel_Response_

// alias to use template instance with default allocator
using SetChassisMaxVel_Response =
  controller::srv::SetChassisMaxVel_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct SetChassisMaxVel
{
  using Request = controller::srv::SetChassisMaxVel_Request;
  using Response = controller::srv::SetChassisMaxVel_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__STRUCT_HPP_
