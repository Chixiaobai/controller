// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SetTcpPayload.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_TCP_PAYLOAD__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SET_TCP_PAYLOAD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/set_tcp_payload__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetTcpPayload_Request_tcp_payload_params
{
public:
  Init_SetTcpPayload_Request_tcp_payload_params()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetTcpPayload_Request tcp_payload_params(::controller::srv::SetTcpPayload_Request::_tcp_payload_params_type arg)
  {
    msg_.tcp_payload_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetTcpPayload_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetTcpPayload_Request>()
{
  return controller::srv::builder::Init_SetTcpPayload_Request_tcp_payload_params();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetTcpPayload_Response_success
{
public:
  Init_SetTcpPayload_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetTcpPayload_Response success(::controller::srv::SetTcpPayload_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetTcpPayload_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetTcpPayload_Response>()
{
  return controller::srv::builder::Init_SetTcpPayload_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_TCP_PAYLOAD__BUILDER_HPP_
