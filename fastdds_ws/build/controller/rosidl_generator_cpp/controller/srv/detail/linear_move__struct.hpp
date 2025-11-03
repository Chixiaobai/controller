// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/LinearMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__LINEAR_MOVE__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__LINEAR_MOVE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'linear_move'
#include "controller/msg/detail/linear_move_params__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__srv__LinearMove_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__LinearMove_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LinearMove_Request_
{
  using Type = LinearMove_Request_<ContainerAllocator>;

  explicit LinearMove_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit LinearMove_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _linear_move_type =
    std::vector<controller::msg::LinearMoveParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::LinearMoveParams_<ContainerAllocator>>>;
  _linear_move_type linear_move;

  // setters for named parameter idiom
  Type & set__linear_move(
    const std::vector<controller::msg::LinearMoveParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::LinearMoveParams_<ContainerAllocator>>> & _arg)
  {
    this->linear_move = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::LinearMove_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::LinearMove_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::LinearMove_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::LinearMove_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearMove_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearMove_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearMove_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearMove_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::LinearMove_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::LinearMove_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__LinearMove_Request
    std::shared_ptr<controller::srv::LinearMove_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__LinearMove_Request
    std::shared_ptr<controller::srv::LinearMove_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearMove_Request_ & other) const
  {
    if (this->linear_move != other.linear_move) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearMove_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearMove_Request_

// alias to use template instance with default allocator
using LinearMove_Request =
  controller::srv::LinearMove_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__LinearMove_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__LinearMove_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LinearMove_Response_
{
  using Type = LinearMove_Response_<ContainerAllocator>;

  explicit LinearMove_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0l;
      this->task_id = 0l;
    }
  }

  explicit LinearMove_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->result = 0l;
      this->task_id = 0l;
    }
  }

  // field types and members
  using _result_type =
    int32_t;
  _result_type result;
  using _task_id_type =
    int32_t;
  _task_id_type task_id;

  // setters for named parameter idiom
  Type & set__result(
    const int32_t & _arg)
  {
    this->result = _arg;
    return *this;
  }
  Type & set__task_id(
    const int32_t & _arg)
  {
    this->task_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::LinearMove_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::LinearMove_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::LinearMove_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::LinearMove_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearMove_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearMove_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::LinearMove_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::LinearMove_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::LinearMove_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::LinearMove_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__LinearMove_Response
    std::shared_ptr<controller::srv::LinearMove_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__LinearMove_Response
    std::shared_ptr<controller::srv::LinearMove_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LinearMove_Response_ & other) const
  {
    if (this->result != other.result) {
      return false;
    }
    if (this->task_id != other.task_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const LinearMove_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LinearMove_Response_

// alias to use template instance with default allocator
using LinearMove_Response =
  controller::srv::LinearMove_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct LinearMove
{
  using Request = controller::srv::LinearMove_Request;
  using Response = controller::srv::LinearMove_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__LINEAR_MOVE__STRUCT_HPP_
