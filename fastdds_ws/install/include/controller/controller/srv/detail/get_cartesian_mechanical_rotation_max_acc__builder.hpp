// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetCartesianMechanicalRotationMaxAcc.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_CARTESIAN_MECHANICAL_ROTATION_MAX_ACC__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_CARTESIAN_MECHANICAL_ROTATION_MAX_ACC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_cartesian_mechanical_rotation_max_acc__struct.hpp"
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
auto build<::controller::srv::GetCartesianMechanicalRotationMaxAcc_Request>()
{
  return ::controller::srv::GetCartesianMechanicalRotationMaxAcc_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetCartesianMechanicalRotationMaxAcc_Response_cartesian_params
{
public:
  Init_GetCartesianMechanicalRotationMaxAcc_Response_cartesian_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::GetCartesianMechanicalRotationMaxAcc_Response cartesian_params(::controller::srv::GetCartesianMechanicalRotationMaxAcc_Response::_cartesian_params_type arg)
  {
    msg_.cartesian_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetCartesianMechanicalRotationMaxAcc_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetCartesianMechanicalRotationMaxAcc_Response>()
{
  return controller::srv::builder::Init_GetCartesianMechanicalRotationMaxAcc_Response_cartesian_params();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_CARTESIAN_MECHANICAL_ROTATION_MAX_ACC__BUILDER_HPP_
