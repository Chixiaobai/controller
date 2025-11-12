// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetJointMechanicalLimit.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_LIMIT__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_LIMIT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_joint_mechanical_limit__struct.hpp"
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
auto build<::controller::srv::GetJointMechanicalLimit_Request>()
{
  return ::controller::srv::GetJointMechanicalLimit_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetJointMechanicalLimit_Response_joint_params
{
public:
  Init_GetJointMechanicalLimit_Response_joint_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::GetJointMechanicalLimit_Response joint_params(::controller::srv::GetJointMechanicalLimit_Response::_joint_params_type arg)
  {
    msg_.joint_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetJointMechanicalLimit_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetJointMechanicalLimit_Response>()
{
  return controller::srv::builder::Init_GetJointMechanicalLimit_Response_joint_params();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_LIMIT__BUILDER_HPP_
