// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/ErrorMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__ErrorMessage __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__ErrorMessage __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ErrorMessage_
{
  using Type = ErrorMessage_<ContainerAllocator>;

  explicit ErrorMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->level = 0;
      this->module = 0;
      this->error_code = 0l;
      this->msg = "";
    }
  }

  explicit ErrorMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : msg(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->level = 0;
      this->module = 0;
      this->error_code = 0l;
      this->msg = "";
    }
  }

  // field types and members
  using _id_type =
    uint32_t;
  _id_type id;
  using _level_type =
    uint8_t;
  _level_type level;
  using _module_type =
    uint8_t;
  _module_type module;
  using _error_code_type =
    int32_t;
  _error_code_type error_code;
  using _msg_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _msg_type msg;

  // setters for named parameter idiom
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__level(
    const uint8_t & _arg)
  {
    this->level = _arg;
    return *this;
  }
  Type & set__module(
    const uint8_t & _arg)
  {
    this->module = _arg;
    return *this;
  }
  Type & set__error_code(
    const int32_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__msg(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->msg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::ErrorMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::ErrorMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::ErrorMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::ErrorMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::ErrorMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::ErrorMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::ErrorMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::ErrorMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::ErrorMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::ErrorMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__ErrorMessage
    std::shared_ptr<controller::msg::ErrorMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__ErrorMessage
    std::shared_ptr<controller::msg::ErrorMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ErrorMessage_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->level != other.level) {
      return false;
    }
    if (this->module != other.module) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->msg != other.msg) {
      return false;
    }
    return true;
  }
  bool operator!=(const ErrorMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ErrorMessage_

// alias to use template instance with default allocator
using ErrorMessage =
  controller::msg::ErrorMessage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__STRUCT_HPP_
