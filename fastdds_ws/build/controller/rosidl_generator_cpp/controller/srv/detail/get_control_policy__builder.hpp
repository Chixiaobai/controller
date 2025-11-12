// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetControlPolicy.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_CONTROL_POLICY__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_CONTROL_POLICY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_control_policy__struct.hpp"
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
auto build<::controller::srv::GetControlPolicy_Request>()
{
  return ::controller::srv::GetControlPolicy_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetControlPolicy_Response_success
{
public:
  explicit Init_GetControlPolicy_Response_success(::controller::srv::GetControlPolicy_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::GetControlPolicy_Response success(::controller::srv::GetControlPolicy_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetControlPolicy_Response msg_;
};

class Init_GetControlPolicy_Response_policy
{
public:
  Init_GetControlPolicy_Response_policy()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetControlPolicy_Response_success policy(::controller::srv::GetControlPolicy_Response::_policy_type arg)
  {
    msg_.policy = std::move(arg);
    return Init_GetControlPolicy_Response_success(msg_);
  }

private:
  ::controller::srv::GetControlPolicy_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetControlPolicy_Response>()
{
  return controller::srv::builder::Init_GetControlPolicy_Response_policy();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_CONTROL_POLICY__BUILDER_HPP_
