// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/IsEnabledController.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__IS_ENABLED_CONTROLLER__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__IS_ENABLED_CONTROLLER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/is_enabled_controller__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::IsEnabledController_Request>()
{
  return ::controller::srv::IsEnabledController_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_IsEnabledController_Response_success
{
public:
  explicit Init_IsEnabledController_Response_success(::controller::srv::IsEnabledController_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::IsEnabledController_Response success(::controller::srv::IsEnabledController_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::IsEnabledController_Response msg_;
};

class Init_IsEnabledController_Response_enable
{
public:
  Init_IsEnabledController_Response_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IsEnabledController_Response_success enable(::controller::srv::IsEnabledController_Response::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return Init_IsEnabledController_Response_success(msg_);
  }

private:
  ::controller::srv::IsEnabledController_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::IsEnabledController_Response>()
{
  return controller::srv::builder::Init_IsEnabledController_Response_enable();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__IS_ENABLED_CONTROLLER__BUILDER_HPP_
