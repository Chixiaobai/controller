// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SetJointSoftLimit.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_JOINT_SOFT_LIMIT__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SET_JOINT_SOFT_LIMIT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/set_joint_soft_limit__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetJointSoftLimit_Request_joint_params
{
public:
  Init_SetJointSoftLimit_Request_joint_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetJointSoftLimit_Request joint_params(::controller::srv::SetJointSoftLimit_Request::_joint_params_type arg)
  {
    msg_.joint_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetJointSoftLimit_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetJointSoftLimit_Request>()
{
  return controller::srv::builder::Init_SetJointSoftLimit_Request_joint_params();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetJointSoftLimit_Response_success
{
public:
  Init_SetJointSoftLimit_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetJointSoftLimit_Response success(::controller::srv::SetJointSoftLimit_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetJointSoftLimit_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetJointSoftLimit_Response>()
{
  return controller::srv::builder::Init_SetJointSoftLimit_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_JOINT_SOFT_LIMIT__BUILDER_HPP_
