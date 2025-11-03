// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from controller:msg/CartesianParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__STRUCT_HPP_
#define CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__controller__msg__CartesianParams __attribute__((deprecated))
#else
# define DEPRECATED__controller__msg__CartesianParams __declspec(deprecated)
#endif

namespace controller
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CartesianParams_
{
  using Type = CartesianParams_<ContainerAllocator>;

  explicit CartesianParams_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cartesian_index = 0ul;
      this->trans_max_vel = 0.0;
      this->trans_max_acc = 0.0;
      this->rota_max_vel = 0.0;
      this->rota_max_acc = 0.0;
    }
  }

  explicit CartesianParams_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cartesian_index = 0ul;
      this->trans_max_vel = 0.0;
      this->trans_max_acc = 0.0;
      this->rota_max_vel = 0.0;
      this->rota_max_acc = 0.0;
    }
  }

  // field types and members
  using _cartesian_index_type =
    uint32_t;
  _cartesian_index_type cartesian_index;
  using _trans_max_vel_type =
    double;
  _trans_max_vel_type trans_max_vel;
  using _trans_max_acc_type =
    double;
  _trans_max_acc_type trans_max_acc;
  using _rota_max_vel_type =
    double;
  _rota_max_vel_type rota_max_vel;
  using _rota_max_acc_type =
    double;
  _rota_max_acc_type rota_max_acc;

  // setters for named parameter idiom
  Type & set__cartesian_index(
    const uint32_t & _arg)
  {
    this->cartesian_index = _arg;
    return *this;
  }
  Type & set__trans_max_vel(
    const double & _arg)
  {
    this->trans_max_vel = _arg;
    return *this;
  }
  Type & set__trans_max_acc(
    const double & _arg)
  {
    this->trans_max_acc = _arg;
    return *this;
  }
  Type & set__rota_max_vel(
    const double & _arg)
  {
    this->rota_max_vel = _arg;
    return *this;
  }
  Type & set__rota_max_acc(
    const double & _arg)
  {
    this->rota_max_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    controller::msg::CartesianParams_<ContainerAllocator> *;
  using ConstRawPtr =
    const controller::msg::CartesianParams_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<controller::msg::CartesianParams_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<controller::msg::CartesianParams_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      controller::msg::CartesianParams_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<controller::msg::CartesianParams_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      controller::msg::CartesianParams_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<controller::msg::CartesianParams_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<controller::msg::CartesianParams_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<controller::msg::CartesianParams_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__controller__msg__CartesianParams
    std::shared_ptr<controller::msg::CartesianParams_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__controller__msg__CartesianParams
    std::shared_ptr<controller::msg::CartesianParams_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CartesianParams_ & other) const
  {
    if (this->cartesian_index != other.cartesian_index) {
      return false;
    }
    if (this->trans_max_vel != other.trans_max_vel) {
      return false;
    }
    if (this->trans_max_acc != other.trans_max_acc) {
      return false;
    }
    if (this->rota_max_vel != other.rota_max_vel) {
      return false;
    }
    if (this->rota_max_acc != other.rota_max_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const CartesianParams_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CartesianParams_

// alias to use template instance with default allocator
using CartesianParams =
  controller::msg::CartesianParams_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__STRUCT_HPP_
