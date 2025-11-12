// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetTcpOffset.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_tcp_offset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetTcpOffset_Request_type
{
public:
  Init_GetTcpOffset_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::GetTcpOffset_Request type(::controller::srv::GetTcpOffset_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetTcpOffset_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetTcpOffset_Request>()
{
  return controller::srv::builder::Init_GetTcpOffset_Request_type();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetTcpOffset_Response_tcp_offset_params
{
public:
  explicit Init_GetTcpOffset_Response_tcp_offset_params(::controller::srv::GetTcpOffset_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::GetTcpOffset_Response tcp_offset_params(::controller::srv::GetTcpOffset_Response::_tcp_offset_params_type arg)
  {
    msg_.tcp_offset_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetTcpOffset_Response msg_;
};

class Init_GetTcpOffset_Response_success
{
public:
  Init_GetTcpOffset_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTcpOffset_Response_tcp_offset_params success(::controller::srv::GetTcpOffset_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetTcpOffset_Response_tcp_offset_params(msg_);
  }

private:
  ::controller::srv::GetTcpOffset_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetTcpOffset_Response>()
{
  return controller::srv::builder::Init_GetTcpOffset_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__BUILDER_HPP_
