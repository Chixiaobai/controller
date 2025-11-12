// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SetCartesianTranslationMaxAcc.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_ACC__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_ACC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/set_cartesian_translation_max_acc__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetCartesianTranslationMaxAcc_Request_cartesian_params
{
public:
  Init_SetCartesianTranslationMaxAcc_Request_cartesian_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetCartesianTranslationMaxAcc_Request cartesian_params(::controller::srv::SetCartesianTranslationMaxAcc_Request::_cartesian_params_type arg)
  {
    msg_.cartesian_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetCartesianTranslationMaxAcc_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetCartesianTranslationMaxAcc_Request>()
{
  return controller::srv::builder::Init_SetCartesianTranslationMaxAcc_Request_cartesian_params();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetCartesianTranslationMaxAcc_Response_success
{
public:
  Init_SetCartesianTranslationMaxAcc_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetCartesianTranslationMaxAcc_Response success(::controller::srv::SetCartesianTranslationMaxAcc_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetCartesianTranslationMaxAcc_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetCartesianTranslationMaxAcc_Response>()
{
  return controller::srv::builder::Init_SetCartesianTranslationMaxAcc_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_ACC__BUILDER_HPP_
