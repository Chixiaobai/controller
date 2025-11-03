// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetJointMechanicalMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_MAX_VEL__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_MAX_VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_joint_mechanical_max_vel__struct.hpp"
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
auto build<::controller::srv::GetJointMechanicalMaxVel_Request>()
{
  return ::controller::srv::GetJointMechanicalMaxVel_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetJointMechanicalMaxVel_Response_joint_params
{
public:
  Init_GetJointMechanicalMaxVel_Response_joint_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::GetJointMechanicalMaxVel_Response joint_params(::controller::srv::GetJointMechanicalMaxVel_Response::_joint_params_type arg)
  {
    msg_.joint_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetJointMechanicalMaxVel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetJointMechanicalMaxVel_Response>()
{
  return controller::srv::builder::Init_GetJointMechanicalMaxVel_Response_joint_params();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_MAX_VEL__BUILDER_HPP_
