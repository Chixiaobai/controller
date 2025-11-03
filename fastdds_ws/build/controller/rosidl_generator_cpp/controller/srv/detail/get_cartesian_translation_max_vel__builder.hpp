// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetCartesianTranslationMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_CARTESIAN_TRANSLATION_MAX_VEL__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_CARTESIAN_TRANSLATION_MAX_VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_cartesian_translation_max_vel__struct.hpp"
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
auto build<::controller::srv::GetCartesianTranslationMaxVel_Request>()
{
  return ::controller::srv::GetCartesianTranslationMaxVel_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetCartesianTranslationMaxVel_Response_cartesian_params
{
public:
  Init_GetCartesianTranslationMaxVel_Response_cartesian_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::GetCartesianTranslationMaxVel_Response cartesian_params(::controller::srv::GetCartesianTranslationMaxVel_Response::_cartesian_params_type arg)
  {
    msg_.cartesian_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetCartesianTranslationMaxVel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetCartesianTranslationMaxVel_Response>()
{
  return controller::srv::builder::Init_GetCartesianTranslationMaxVel_Response_cartesian_params();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_CARTESIAN_TRANSLATION_MAX_VEL__BUILDER_HPP_
