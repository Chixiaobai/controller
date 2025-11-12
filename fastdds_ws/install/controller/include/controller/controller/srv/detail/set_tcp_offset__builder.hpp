// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SetTcpOffset.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_TCP_OFFSET__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SET_TCP_OFFSET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/set_tcp_offset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetTcpOffset_Request_tcp_offset_params
{
public:
  Init_SetTcpOffset_Request_tcp_offset_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetTcpOffset_Request tcp_offset_params(::controller::srv::SetTcpOffset_Request::_tcp_offset_params_type arg)
  {
    msg_.tcp_offset_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetTcpOffset_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetTcpOffset_Request>()
{
  return controller::srv::builder::Init_SetTcpOffset_Request_tcp_offset_params();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetTcpOffset_Response_success
{
public:
  Init_SetTcpOffset_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetTcpOffset_Response success(::controller::srv::SetTcpOffset_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetTcpOffset_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetTcpOffset_Response>()
{
  return controller::srv::builder::Init_SetTcpOffset_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_TCP_OFFSET__BUILDER_HPP_
