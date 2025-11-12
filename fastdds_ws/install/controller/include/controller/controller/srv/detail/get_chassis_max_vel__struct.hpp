// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/GetChassisMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_CHASSIS_MAX_VEL__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__GET_CHASSIS_MAX_VEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__GetChassisMaxVel_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__GetChassisMaxVel_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetChassisMaxVel_Request_
{
  using Type = GetChassisMaxVel_Request_<ContainerAllocator>;

  explicit GetChassisMaxVel_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetChassisMaxVel_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::GetChassisMaxVel_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::GetChassisMaxVel_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::GetChassisMaxVel_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::GetChassisMaxVel_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetChassisMaxVel_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetChassisMaxVel_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetChassisMaxVel_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetChassisMaxVel_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::GetChassisMaxVel_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::GetChassisMaxVel_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__GetChassisMaxVel_Request
    std::shared_ptr<controller::srv::GetChassisMaxVel_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__GetChassisMaxVel_Request
    std::shared_ptr<controller::srv::GetChassisMaxVel_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetChassisMaxVel_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetChassisMaxVel_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetChassisMaxVel_Request_

// alias to use template instance with default allocator
using GetChassisMaxVel_Request =
  controller::srv::GetChassisMaxVel_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__GetChassisMaxVel_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__GetChassisMaxVel_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetChassisMaxVel_Response_
{
  using Type = GetChassisMaxVel_Response_<ContainerAllocator>;

  explicit GetChassisMaxVel_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0;
      this->angular_velocity = 0.0;
    }
  }

  explicit GetChassisMaxVel_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    controller::srv::GetChassisMaxVel_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::GetChassisMaxVel_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::GetChassisMaxVel_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::GetChassisMaxVel_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetChassisMaxVel_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetChassisMaxVel_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetChassisMaxVel_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetChassisMaxVel_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::GetChassisMaxVel_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::GetChassisMaxVel_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__GetChassisMaxVel_Response
    std::shared_ptr<controller::srv::GetChassisMaxVel_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__GetChassisMaxVel_Response
    std::shared_ptr<controller::srv::GetChassisMaxVel_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetChassisMaxVel_Response_ & other) const
  {
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetChassisMaxVel_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetChassisMaxVel_Response_

// alias to use template instance with default allocator
using GetChassisMaxVel_Response =
  controller::srv::GetChassisMaxVel_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct GetChassisMaxVel
{
  using Request = controller::srv::GetChassisMaxVel_Request;
  using Response = controller::srv::GetChassisMaxVel_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_CHASSIS_MAX_VEL__STRUCT_HPP_
