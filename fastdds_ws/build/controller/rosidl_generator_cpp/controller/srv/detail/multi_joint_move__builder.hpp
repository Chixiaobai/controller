// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/MultiJointMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/multi_joint_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_MultiJointMove_Request_joint_angles
{
public:
  Init_MultiJointMove_Request_joint_angles()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::MultiJointMove_Request joint_angles(::controller::srv::MultiJointMove_Request::_joint_angles_type arg)
  {
    msg_.joint_angles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::MultiJointMove_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::MultiJointMove_Request>()
{
  return controller::srv::builder::Init_MultiJointMove_Request_joint_angles();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_MultiJointMove_Response_token
{
public:
  explicit Init_MultiJointMove_Response_token(::controller::srv::MultiJointMove_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::MultiJointMove_Response token(::controller::srv::MultiJointMove_Response::_token_type arg)
  {
    msg_.token = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::MultiJointMove_Response msg_;
};

class Init_MultiJointMove_Response_success
{
public:
  Init_MultiJointMove_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiJointMove_Response_token success(::controller::srv::MultiJointMove_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_MultiJointMove_Response_token(msg_);
  }

private:
  ::controller::srv::MultiJointMove_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::MultiJointMove_Response>()
{
  return controller::srv::builder::Init_MultiJointMove_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__BUILDER_HPP_
