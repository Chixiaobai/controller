// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/GetTcpOffset.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__GetTcpOffset_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__GetTcpOffset_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTcpOffset_Request_
{
  using Type = GetTcpOffset_Request_<ContainerAllocator>;

  explicit GetTcpOffset_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit GetTcpOffset_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _type_type =
    std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>>;
  _type_type type;

  // setters for named parameter idiom
  Type & set__type(
    const std::vector<int32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int32_t>> & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::GetTcpOffset_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::GetTcpOffset_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::GetTcpOffset_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::GetTcpOffset_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetTcpOffset_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetTcpOffset_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetTcpOffset_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetTcpOffset_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::GetTcpOffset_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::GetTcpOffset_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__GetTcpOffset_Request
    std::shared_ptr<controller::srv::GetTcpOffset_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__GetTcpOffset_Request
    std::shared_ptr<controller::srv::GetTcpOffset_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTcpOffset_Request_ & other) const
  {
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTcpOffset_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTcpOffset_Request_

// alias to use template instance with default allocator
using GetTcpOffset_Request =
  controller::srv::GetTcpOffset_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


// Include directives for member types
// Member 'tcp_offset_params'
#include "controller/msg/detail/tcp_offset_params__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__srv__GetTcpOffset_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__GetTcpOffset_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTcpOffset_Response_
{
  using Type = GetTcpOffset_Response_<ContainerAllocator>;

  explicit GetTcpOffset_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit GetTcpOffset_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
  using _tcp_offset_params_type =
    std::vector<controller::msg::TcpOffsetParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpOffsetParams_<ContainerAllocator>>>;
  _tcp_offset_params_type tcp_offset_params;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__tcp_offset_params(
    const std::vector<controller::msg::TcpOffsetParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::TcpOffsetParams_<ContainerAllocator>>> & _arg)
  {
    this->tcp_offset_params = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::GetTcpOffset_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::GetTcpOffset_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::GetTcpOffset_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::GetTcpOffset_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetTcpOffset_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetTcpOffset_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetTcpOffset_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetTcpOffset_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::GetTcpOffset_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::GetTcpOffset_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__GetTcpOffset_Response
    std::shared_ptr<controller::srv::GetTcpOffset_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__GetTcpOffset_Response
    std::shared_ptr<controller::srv::GetTcpOffset_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTcpOffset_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->tcp_offset_params != other.tcp_offset_params) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTcpOffset_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTcpOffset_Response_

// alias to use template instance with default allocator
using GetTcpOffset_Response =
  controller::srv::GetTcpOffset_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct GetTcpOffset
{
  using Request = controller::srv::GetTcpOffset_Request;
  using Response = controller::srv::GetTcpOffset_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__STRUCT_HPP_
