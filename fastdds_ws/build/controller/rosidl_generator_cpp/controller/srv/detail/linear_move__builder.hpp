// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/LinearMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__LINEAR_MOVE__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__LINEAR_MOVE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/linear_move__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_LinearMove_Request_linear_move
{
public:
  Init_LinearMove_Request_linear_move()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::LinearMove_Request linear_move(::controller::srv::LinearMove_Request::_linear_move_type arg)
  {
    msg_.linear_move = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::LinearMove_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::LinearMove_Request>()
{
  return controller::srv::builder::Init_LinearMove_Request_linear_move();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_LinearMove_Response_token
{
public:
  explicit Init_LinearMove_Response_token(::controller::srv::LinearMove_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::LinearMove_Response token(::controller::srv::LinearMove_Response::_token_type arg)
  {
    msg_.token = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::LinearMove_Response msg_;
};

class Init_LinearMove_Response_success
{
public:
  Init_LinearMove_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearMove_Response_token success(::controller::srv::LinearMove_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_LinearMove_Response_token(msg_);
  }

private:
  ::controller::srv::LinearMove_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::LinearMove_Response>()
{
  return controller::srv::builder::Init_LinearMove_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__LINEAR_MOVE__BUILDER_HPP_
