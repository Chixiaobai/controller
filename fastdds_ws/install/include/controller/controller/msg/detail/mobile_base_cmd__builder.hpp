// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/MobileBaseCmd.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/mobile_base_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_MobileBaseCmd_lifetime
{
public:
  explicit Init_MobileBaseCmd_lifetime(::controller::msg::MobileBaseCmd & msg)
  : msg_(msg)
  {}
  ::controller::msg::MobileBaseCmd lifetime(::controller::msg::MobileBaseCmd::_lifetime_type arg)
  {
    msg_.lifetime = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::MobileBaseCmd msg_;
};

class Init_MobileBaseCmd_angular_velocity
{
public:
  explicit Init_MobileBaseCmd_angular_velocity(::controller::msg::MobileBaseCmd & msg)
  : msg_(msg)
  {}
  Init_MobileBaseCmd_lifetime angular_velocity(::controller::msg::MobileBaseCmd::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_MobileBaseCmd_lifetime(msg_);
  }

private:
  ::controller::msg::MobileBaseCmd msg_;
};

class Init_MobileBaseCmd_linear_velocity
{
public:
  Init_MobileBaseCmd_linear_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MobileBaseCmd_angular_velocity linear_velocity(::controller::msg::MobileBaseCmd::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_MobileBaseCmd_angular_velocity(msg_);
  }

private:
  ::controller::msg::MobileBaseCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::MobileBaseCmd>()
{
  return controller::msg::builder::Init_MobileBaseCmd_linear_velocity();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__BUILDER_HPP_
