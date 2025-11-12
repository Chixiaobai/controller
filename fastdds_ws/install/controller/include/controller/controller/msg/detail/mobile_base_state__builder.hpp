// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/MobileBaseState.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/mobile_base_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_MobileBaseState_angular_velocity
{
public:
  explicit Init_MobileBaseState_angular_velocity(::controller::msg::MobileBaseState & msg)
  : msg_(msg)
  {}
  ::controller::msg::MobileBaseState angular_velocity(::controller::msg::MobileBaseState::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::MobileBaseState msg_;
};

class Init_MobileBaseState_linear_velocity
{
public:
  Init_MobileBaseState_linear_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MobileBaseState_angular_velocity linear_velocity(::controller::msg::MobileBaseState::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_MobileBaseState_angular_velocity(msg_);
  }

private:
  ::controller::msg::MobileBaseState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::MobileBaseState>()
{
  return controller::msg::builder::Init_MobileBaseState_linear_velocity();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__BUILDER_HPP_
