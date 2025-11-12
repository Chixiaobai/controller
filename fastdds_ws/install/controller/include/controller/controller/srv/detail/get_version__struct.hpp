// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:srv/GetVersion.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_VERSION__STRUCT_HPP_
#define CONTROLLER__SRV__DETAIL__GET_VERSION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__srv__GetVersion_Request __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__GetVersion_Request __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetVersion_Request_
{
  using Type = GetVersion_Request_<ContainerAllocator>;

  explicit GetVersion_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetVersion_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    controller::srv::GetVersion_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::GetVersion_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::GetVersion_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::GetVersion_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetVersion_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetVersion_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetVersion_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetVersion_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::GetVersion_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::GetVersion_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__GetVersion_Request
    std::shared_ptr<controller::srv::GetVersion_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__GetVersion_Request
    std::shared_ptr<controller::srv::GetVersion_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetVersion_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetVersion_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetVersion_Request_

// alias to use template instance with default allocator
using GetVersion_Request =
  controller::srv::GetVersion_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller


#ifndef _WIN32
# define DEPRECATED__controller__srv__GetVersion_Response __attribute__((deprecated))
#else
# define DEPRECATED__controller__srv__GetVersion_Response __declspec(deprecated)
#endif

namespace controller
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetVersion_Response_
{
  using Type = GetVersion_Response_<ContainerAllocator>;

  explicit GetVersion_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->main = "";
    }
  }

  explicit GetVersion_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : main(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->main = "";
    }
  }

  // field types and members
  using _main_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _main_type main;
  using _plugin_names_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _plugin_names_type plugin_names;
  using _plugin_versions_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _plugin_versions_type plugin_versions;

  // setters for named parameter idiom
  Type & set__main(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->main = _arg;
    return *this;
  }
  Type & set__plugin_names(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->plugin_names = _arg;
    return *this;
  }
  Type & set__plugin_versions(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->plugin_versions = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::srv::GetVersion_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::srv::GetVersion_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::srv::GetVersion_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::srv::GetVersion_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetVersion_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetVersion_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::srv::GetVersion_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::srv::GetVersion_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::srv::GetVersion_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::srv::GetVersion_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__srv__GetVersion_Response
    std::shared_ptr<controller::srv::GetVersion_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__srv__GetVersion_Response
    std::shared_ptr<controller::srv::GetVersion_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetVersion_Response_ & other) const
  {
    if (this->main != other.main) {
      return false;
    }
    if (this->plugin_names != other.plugin_names) {
      return false;
    }
    if (this->plugin_versions != other.plugin_versions) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetVersion_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetVersion_Response_

// alias to use template instance with default allocator
using GetVersion_Response =
  controller::srv::GetVersion_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace controller

namespace controller
{

namespace srv
{

struct GetVersion
{
  using Request = controller::srv::GetVersion_Request;
  using Response = controller::srv::GetVersion_Response;
};

}  // namespace srv

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_VERSION__STRUCT_HPP_
