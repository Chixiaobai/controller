// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SetJointMaxAcc.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_JOINT_MAX_ACC__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SET_JOINT_MAX_ACC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/set_joint_max_acc__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetJointMaxAcc_Request_joint_params
{
public:
  Init_SetJointMaxAcc_Request_joint_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetJointMaxAcc_Request joint_params(::controller::srv::SetJointMaxAcc_Request::_joint_params_type arg)
  {
    msg_.joint_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetJointMaxAcc_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetJointMaxAcc_Request>()
{
  return controller::srv::builder::Init_SetJointMaxAcc_Request_joint_params();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetJointMaxAcc_Response_success
{
public:
  Init_SetJointMaxAcc_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetJointMaxAcc_Response success(::controller::srv::SetJointMaxAcc_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetJointMaxAcc_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetJointMaxAcc_Response>()
{
  return controller::srv::builder::Init_SetJointMaxAcc_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_JOINT_MAX_ACC__BUILDER_HPP_
