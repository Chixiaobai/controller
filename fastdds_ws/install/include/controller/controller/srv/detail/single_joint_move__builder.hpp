// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SingleJointMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/single_joint_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SingleJointMove_Request_velocity
{
public:
  explicit Init_SingleJointMove_Request_velocity(::controller::srv::SingleJointMove_Request & msg)
  : msg_(msg)
  {}
  ::controller::srv::SingleJointMove_Request velocity(::controller::srv::SingleJointMove_Request::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SingleJointMove_Request msg_;
};

class Init_SingleJointMove_Request_target_position
{
public:
  explicit Init_SingleJointMove_Request_target_position(::controller::srv::SingleJointMove_Request & msg)
  : msg_(msg)
  {}
  Init_SingleJointMove_Request_velocity target_position(::controller::srv::SingleJointMove_Request::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_SingleJointMove_Request_velocity(msg_);
  }

private:
  ::controller::srv::SingleJointMove_Request msg_;
};

class Init_SingleJointMove_Request_joint_index
{
public:
  Init_SingleJointMove_Request_joint_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SingleJointMove_Request_target_position joint_index(::controller::srv::SingleJointMove_Request::_joint_index_type arg)
  {
    msg_.joint_index = std::move(arg);
    return Init_SingleJointMove_Request_target_position(msg_);
  }

private:
  ::controller::srv::SingleJointMove_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SingleJointMove_Request>()
{
  return controller::srv::builder::Init_SingleJointMove_Request_joint_index();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SingleJointMove_Response_token
{
public:
  explicit Init_SingleJointMove_Response_token(::controller::srv::SingleJointMove_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::SingleJointMove_Response token(::controller::srv::SingleJointMove_Response::_token_type arg)
  {
    msg_.token = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SingleJointMove_Response msg_;
};

class Init_SingleJointMove_Response_success
{
public:
  Init_SingleJointMove_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SingleJointMove_Response_token success(::controller::srv::SingleJointMove_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SingleJointMove_Response_token(msg_);
  }

private:
  ::controller::srv::SingleJointMove_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SingleJointMove_Response>()
{
  return controller::srv::builder::Init_SingleJointMove_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__BUILDER_HPP_
