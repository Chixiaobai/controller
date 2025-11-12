// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "controller/msg/detail/tcp_pose_params__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__srv__InverseKinematics_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__InverseKinematics_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InverseKinematics_Request_
{
  using Type = InverseKinematics_Request_<ContainerAllocator>;

  explicit InverseKinematics_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->if_use_whole_body = false;
    }
  }

  explicit InverseKinematics_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->if_use_whole_body = false;
    }
  }

  // field types and members
  using _pose_type =
    std::vector<controller::msg::TcpPoseParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpPoseParams_<ContainerAllocator>>>;
  _pose_type pose;
  using _reference_joint_angles_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _reference_joint_angles_type reference_joint_angles;
  using _if_use_whole_body_type =
    bool;
  _if_use_whole_body_type if_use_whole_body;

  // setters for named parameter idiom
  Type & set__pose(
    const std::vector<controller::msg::TcpPoseParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpPoseParams_<ContainerAllocator>>> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__reference_joint_angles(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->reference_joint_angles = _arg;
    return *this;
  }
  Type & set__if_use_whole_body(
    const bool & _arg)
  {
    this->if_use_whole_body = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::InverseKinematics_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::InverseKinematics_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::InverseKinematics_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::InverseKinematics_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::InverseKinematics_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::InverseKinematics_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::InverseKinematics_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::InverseKinematics_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::InverseKinematics_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::InverseKinematics_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__InverseKinematics_Request
    std::shared_ptr<controller::srv::InverseKinematics_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__InverseKinematics_Request
    std::shared_ptr<controller::srv::InverseKinematics_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InverseKinematics_Request_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->reference_joint_angles != other.reference_joint_angles) {
      return false;
    }
    if (this->if_use_whole_body != other.if_use_whole_body) {
      return false;
    }
    return true;
  }
  bool operator!=(const InverseKinematics_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InverseKinematics_Request_

// alias to use template instance with default allocator
using InverseKinematics_Request =
  controller::srv::InverseKinematics_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__InverseKinematics_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__InverseKinematics_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct InverseKinematics_Response_
{
  using Type = InverseKinematics_Response_<ContainerAllocator>;

  explicit InverseKinematics_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit InverseKinematics_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _joint_angles_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_angles_type joint_angles;

  // setters for named parameter idiom
  Type & set__joint_angles(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_angles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::InverseKinematics_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::InverseKinematics_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::InverseKinematics_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::InverseKinematics_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::InverseKinematics_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::InverseKinematics_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::InverseKinematics_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::InverseKinematics_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::InverseKinematics_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::InverseKinematics_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__InverseKinematics_Response
    std::shared_ptr<controller::srv::InverseKinematics_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__InverseKinematics_Response
    std::shared_ptr<controller::srv::InverseKinematics_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InverseKinematics_Response_ & other) const
  {
    if (this->joint_angles != other.joint_angles) {
      return false;
    }
    return true;
  }
  bool operator!=(const InverseKinematics_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InverseKinematics_Response_

// alias to use template instance with default allocator
using InverseKinematics_Response =
  controller::srv::InverseKinematics_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct InverseKinematics
{
  using Request = controller::srv::InverseKinematics_Request;
  using Response = controller::srv::InverseKinematics_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_HPP_
