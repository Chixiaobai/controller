// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SetControlPolicy.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CONTROL_POLICY__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CONTROL_POLICY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/set_control_policy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetControlPolicy_Request_policy
{
public:
  Init_SetControlPolicy_Request_policy()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetControlPolicy_Request policy(::controller::srv::SetControlPolicy_Request::_policy_type arg)
  {
    msg_.policy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetControlPolicy_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetControlPolicy_Request>()
{
  return controller::srv::builder::Init_SetControlPolicy_Request_policy();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetControlPolicy_Response_success
{
public:
  Init_SetControlPolicy_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetControlPolicy_Response success(::controller::srv::SetControlPolicy_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetControlPolicy_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetControlPolicy_Response>()
{
  return controller::srv::builder::Init_SetControlPolicy_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_CONTROL_POLICY__BUILDER_HPP_
