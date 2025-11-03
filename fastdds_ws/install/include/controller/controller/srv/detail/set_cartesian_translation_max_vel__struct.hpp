// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/SetCartesianTranslationMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_VEL__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_VEL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'cartesian_params'
#include "controller/msg/detail/cartesian_params__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__controller__srv__SetCartesianTranslationMaxVel_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SetCartesianTranslationMaxVel_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetCartesianTranslationMaxVel_Request_
{
  using Type = SetCartesianTranslationMaxVel_Request_<ContainerAllocator>;

  explicit SetCartesianTranslationMaxVel_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit SetCartesianTranslationMaxVel_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _cartesian_params_type =
    std::vector<controller::msg::CartesianParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::CartesianParams_<ContainerAllocator>>>;
  _cartesian_params_type cartesian_params;

  // setters for named parameter idiom
  Type & set__cartesian_params(
    const std::vector<controller::msg::CartesianParams_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<controller::msg::CartesianParams_<ContainerAllocator>>> & _arg)
  {
    this->cartesian_params = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SetCartesianTranslationMaxVel_Request
    std::shared_ptr<controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SetCartesianTranslationMaxVel_Request
    std::shared_ptr<controller::srv::SetCartesianTranslationMaxVel_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetCartesianTranslationMaxVel_Request_ & other) const
  {
    if (this->cartesian_params != other.cartesian_params) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetCartesianTranslationMaxVel_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetCartesianTranslationMaxVel_Request_

// alias to use template instance with default allocator
using SetCartesianTranslationMaxVel_Request =
  controller::srv::SetCartesianTranslationMaxVel_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__SetCartesianTranslationMaxVel_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__SetCartesianTranslationMaxVel_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetCartesianTranslationMaxVel_Response_
{
  using Type = SetCartesianTranslationMaxVel_Response_<ContainerAllocator>;

  explicit SetCartesianTranslationMaxVel_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SetCartesianTranslationMaxVel_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__SetCartesianTranslationMaxVel_Response
    std::shared_ptr<controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__SetCartesianTranslationMaxVel_Response
    std::shared_ptr<controller::srv::SetCartesianTranslationMaxVel_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetCartesianTranslationMaxVel_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetCartesianTranslationMaxVel_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetCartesianTranslationMaxVel_Response_

// alias to use template instance with default allocator
using SetCartesianTranslationMaxVel_Response =
  controller::srv::SetCartesianTranslationMaxVel_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct SetCartesianTranslationMaxVel
{
  using Request = controller::srv::SetCartesianTranslationMaxVel_Request;
  using Response = controller::srv::SetCartesianTranslationMaxVel_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_VEL__STRUCT_HPP_
