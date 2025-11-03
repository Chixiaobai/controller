// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/ForwardKinematics.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__ForwardKinematics_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__ForwardKinematics_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ForwardKinematics_Request_
{
  using Type = ForwardKinematics_Request_<ContainerAllocator>;

  explicit ForwardKinematics_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ForwardKinematics_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _type_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _type_type type;
  using _joint_angles_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _joint_angles_type joint_angles;

  // setters for named parameter idiom
  Type & set__type(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__joint_angles(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->joint_angles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::ForwardKinematics_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::ForwardKinematics_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::ForwardKinematics_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::ForwardKinematics_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::ForwardKinematics_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::ForwardKinematics_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::ForwardKinematics_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::ForwardKinematics_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::ForwardKinematics_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::ForwardKinematics_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__ForwardKinematics_Request
    std::shared_ptr<controller::srv::ForwardKinematics_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__ForwardKinematics_Request
    std::shared_ptr<controller::srv::ForwardKinematics_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ForwardKinematics_Request_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    if (this->joint_angles != other.joint_angles) {
      return false;
    }
    return true;
  }
  bool operator!=(const ForwardKinematics_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ForwardKinematics_Request_

// alias to use template instance with default allocator
using ForwardKinematics_Request =
  controller::srv::ForwardKinematics_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


// Include directives for member types
// Member 'pose'
#include "controller/msg/detail/tcp_pose_params__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__srv__ForwardKinematics_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__ForwardKinematics_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ForwardKinematics_Response_
{
  using Type = ForwardKinematics_Response_<ContainerAllocator>;

  explicit ForwardKinematics_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ForwardKinematics_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _pose_type =
    std::vector<controller::msg::TcpPoseParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpPoseParams_<ContainerAllocator>>>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__pose(
    const std::vector<controller::msg::TcpPoseParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpPoseParams_<ContainerAllocator>>> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::ForwardKinematics_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::ForwardKinematics_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::ForwardKinematics_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::ForwardKinematics_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::ForwardKinematics_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::ForwardKinematics_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::ForwardKinematics_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::ForwardKinematics_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::ForwardKinematics_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::ForwardKinematics_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__ForwardKinematics_Response
    std::shared_ptr<controller::srv::ForwardKinematics_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__ForwardKinematics_Response
    std::shared_ptr<controller::srv::ForwardKinematics_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ForwardKinematics_Response_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const ForwardKinematics_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ForwardKinematics_Response_

// alias to use template instance with default allocator
using ForwardKinematics_Response =
  controller::srv::ForwardKinematics_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct ForwardKinematics
{
  using Request = controller::srv::ForwardKinematics_Request;
  using Response = controller::srv::ForwardKinematics_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__STRUCT_HPP_
