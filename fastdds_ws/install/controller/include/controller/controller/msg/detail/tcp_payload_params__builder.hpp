// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/TcpPayloadParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__TCP_PAYLOAD_PARAMS__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__TCP_PAYLOAD_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/tcp_payload_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_TcpPayloadParams_parameters
{
public:
  explicit Init_TcpPayloadParams_parameters(::controller::msg::TcpPayloadParams & msg)
  : msg_(msg)
  {}
  ::controller::msg::TcpPayloadParams parameters(::controller::msg::TcpPayloadParams::_parameters_type arg)
  {
    msg_.parameters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::TcpPayloadParams msg_;
};

class Init_TcpPayloadParams_type
{
public:
  Init_TcpPayloadParams_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TcpPayloadParams_parameters type(::controller::msg::TcpPayloadParams::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_TcpPayloadParams_parameters(msg_);
  }

private:
  ::controller::msg::TcpPayloadParams msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::TcpPayloadParams>()
{
  return controller::msg::builder::Init_TcpPayloadParams_type();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__TCP_PAYLOAD_PARAMS__BUILDER_HPP_
