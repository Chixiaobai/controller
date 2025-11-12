// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetJointMechanicalMaxAcc.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_MAX_ACC__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_MAX_ACC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_joint_mechanical_max_acc__struct.hpp"
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
auto build<::controller::srv::GetJointMechanicalMaxAcc_Request>()
{
  return ::controller::srv::GetJointMechanicalMaxAcc_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetJointMechanicalMaxAcc_Response_joint_params
{
public:
  Init_GetJointMechanicalMaxAcc_Response_joint_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::GetJointMechanicalMaxAcc_Response joint_params(::controller::srv::GetJointMechanicalMaxAcc_Response::_joint_params_type arg)
  {
    msg_.joint_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetJointMechanicalMaxAcc_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetJointMechanicalMaxAcc_Response>()
{
  return controller::srv::builder::Init_GetJointMechanicalMaxAcc_Response_joint_params();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_JOINT_MECHANICAL_MAX_ACC__BUILDER_HPP_
