// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/MultiJointMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'joint_angles'
#include "controller/msg/detail/joint_angle__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__srv__MultiJointMove_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__MultiJointMove_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MultiJointMove_Request_
{
  using Type = MultiJointMove_Request_<ContainerAllocator>;

  explicit MultiJointMove_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MultiJointMove_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _joint_angles_type =
    std::vector<controller::msg::JointAngle_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::JointAngle_<ContainerAllocator>>>;
  _joint_angles_type joint_angles;

  // setters for named parameter idiom
  Type & set__joint_angles(
    const std::vector<controller::msg::JointAngle_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::JointAngle_<ContainerAllocator>>> & _arg)
  {
    this->joint_angles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::MultiJointMove_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::MultiJointMove_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::MultiJointMove_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::MultiJointMove_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::MultiJointMove_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::MultiJointMove_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::MultiJointMove_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::MultiJointMove_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::MultiJointMove_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::MultiJointMove_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__MultiJointMove_Request
    std::shared_ptr<controller::srv::MultiJointMove_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__MultiJointMove_Request
    std::shared_ptr<controller::srv::MultiJointMove_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiJointMove_Request_ & other) const
  {
    if (this->joint_angles != other.joint_angles) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiJointMove_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiJointMove_Request_

// alias to use template instance with default allocator
using MultiJointMove_Request =
  controller::srv::MultiJointMove_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__MultiJointMove_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__MultiJointMove_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MultiJointMove_Response_
{
  using Type = MultiJointMove_Response_<ContainerAllocator>;

  explicit MultiJointMove_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->token = 0ul;
    }
  }

  explicit MultiJointMove_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    controller::srv::MultiJointMove_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::MultiJointMove_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::MultiJointMove_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::MultiJointMove_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::MultiJointMove_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::MultiJointMove_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::MultiJointMove_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::MultiJointMove_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::MultiJointMove_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::MultiJointMove_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__MultiJointMove_Response
    std::shared_ptr<controller::srv::MultiJointMove_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__MultiJointMove_Response
    std::shared_ptr<controller::srv::MultiJointMove_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiJointMove_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->token != other.token) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiJointMove_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiJointMove_Response_

// alias to use template instance with default allocator
using MultiJointMove_Response =
  controller::srv::MultiJointMove_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct MultiJointMove
{
  using Request = controller::srv::MultiJointMove_Request;
  using Response = controller::srv::MultiJointMove_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__STRUCT_HPP_
