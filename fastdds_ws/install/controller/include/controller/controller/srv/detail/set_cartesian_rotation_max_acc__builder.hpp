// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SetCartesianRotationMaxAcc.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CARTESIAN_ROTATION_MAX_ACC__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CARTESIAN_ROTATION_MAX_ACC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/set_cartesian_rotation_max_acc__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetCartesianRotationMaxAcc_Request_cartesian_params
{
public:
  Init_SetCartesianRotationMaxAcc_Request_cartesian_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetCartesianRotationMaxAcc_Request cartesian_params(::controller::srv::SetCartesianRotationMaxAcc_Request::_cartesian_params_type arg)
  {
    msg_.cartesian_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetCartesianRotationMaxAcc_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetCartesianRotationMaxAcc_Request>()
{
  return controller::srv::builder::Init_SetCartesianRotationMaxAcc_Request_cartesian_params();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetCartesianRotationMaxAcc_Response_success
{
public:
  Init_SetCartesianRotationMaxAcc_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetCartesianRotationMaxAcc_Response success(::controller::srv::SetCartesianRotationMaxAcc_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetCartesianRotationMaxAcc_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetCartesianRotationMaxAcc_Response>()
{
  return controller::srv::builder::Init_SetCartesianRotationMaxAcc_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_CARTESIAN_ROTATION_MAX_ACC__BUILDER_HPP_
