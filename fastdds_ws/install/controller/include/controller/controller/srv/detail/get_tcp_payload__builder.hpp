// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetTcpPayload.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_TCP_PAYLOAD__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_TCP_PAYLOAD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_tcp_payload__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetTcpPayload_Request_type
{
public:
  Init_GetTcpPayload_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::GetTcpPayload_Request type(::controller::srv::GetTcpPayload_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetTcpPayload_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetTcpPayload_Request>()
{
  return controller::srv::builder::Init_GetTcpPayload_Request_type();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetTcpPayload_Response_tcp_payload_params
{
public:
  explicit Init_GetTcpPayload_Response_tcp_payload_params(::controller::srv::GetTcpPayload_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::GetTcpPayload_Response tcp_payload_params(::controller::srv::GetTcpPayload_Response::_tcp_payload_params_type arg)
  {
    msg_.tcp_payload_params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetTcpPayload_Response msg_;
};

class Init_GetTcpPayload_Response_success
{
public:
  Init_GetTcpPayload_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTcpPayload_Response_tcp_payload_params success(::controller::srv::GetTcpPayload_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_GetTcpPayload_Response_tcp_payload_params(msg_);
  }

private:
  ::controller::srv::GetTcpPayload_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetTcpPayload_Response>()
{
  return controller::srv::builder::Init_GetTcpPayload_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_TCP_PAYLOAD__BUILDER_HPP_
