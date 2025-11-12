// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/SetTcpOffset.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_TCP_OFFSET__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__SET_TCP_OFFSET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'tcp_offset_params'
#include "controller/msg/detail/tcp_offset_params__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__srv__SetTcpOffset_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SetTcpOffset_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTcpOffset_Request_
{
  using Type = SetTcpOffset_Request_<ContainerAllocator>;

  explicit SetTcpOffset_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SetTcpOffset_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _tcp_offset_params_type =
    std::vector<controller::msg::TcpOffsetParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpOffsetParams_<ContainerAllocator>>>;
  _tcp_offset_params_type tcp_offset_params;

  // setters for named parameter idiom
  Type & set__tcp_offset_params(
    const std::vector<controller::msg::TcpOffsetParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpOffsetParams_<ContainerAllocator>>> & _arg)
  {
    this->tcp_offset_params = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::SetTcpOffset_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SetTcpOffset_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SetTcpOffset_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SetTcpOffset_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetTcpOffset_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetTcpOffset_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetTcpOffset_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetTcpOffset_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SetTcpOffset_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SetTcpOffset_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SetTcpOffset_Request
    std::shared_ptr<controller::srv::SetTcpOffset_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SetTcpOffset_Request
    std::shared_ptr<controller::srv::SetTcpOffset_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTcpOffset_Request_ & other) const
  {
    if (this->tcp_offset_params != other.tcp_offset_params) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTcpOffset_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTcpOffset_Request_

// alias to use template instance with default allocator
using SetTcpOffset_Request =
  controller::srv::SetTcpOffset_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__SetTcpOffset_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SetTcpOffset_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetTcpOffset_Response_
{
  using Type = SetTcpOffset_Response_<ContainerAllocator>;

  explicit SetTcpOffset_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetTcpOffset_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    controller::srv::SetTcpOffset_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SetTcpOffset_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SetTcpOffset_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SetTcpOffset_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetTcpOffset_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetTcpOffset_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetTcpOffset_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetTcpOffset_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SetTcpOffset_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SetTcpOffset_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SetTcpOffset_Response
    std::shared_ptr<controller::srv::SetTcpOffset_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SetTcpOffset_Response
    std::shared_ptr<controller::srv::SetTcpOffset_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetTcpOffset_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetTcpOffset_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetTcpOffset_Response_

// alias to use template instance with default allocator
using SetTcpOffset_Response =
  controller::srv::SetTcpOffset_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct SetTcpOffset
{
  using Request = controller::srv::SetTcpOffset_Request;
  using Response = controller::srv::SetTcpOffset_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_TCP_OFFSET__STRUCT_HPP_
