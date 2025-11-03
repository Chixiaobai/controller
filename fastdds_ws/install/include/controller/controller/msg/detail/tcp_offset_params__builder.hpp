// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/TcpOffsetParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__TCP_OFFSET_PARAMS__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__TCP_OFFSET_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/tcp_offset_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_TcpOffsetParams_offset
{
public:
  explicit Init_TcpOffsetParams_offset(::controller::msg::TcpOffsetParams & msg)
  : msg_(msg)
  {}
  ::controller::msg::TcpOffsetParams offset(::controller::msg::TcpOffsetParams::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::TcpOffsetParams msg_;
};

class Init_TcpOffsetParams_type
{
public:
  Init_TcpOffsetParams_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TcpOffsetParams_offset type(::controller::msg::TcpOffsetParams::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_TcpOffsetParams_offset(msg_);
  }

private:
  ::controller::msg::TcpOffsetParams msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::TcpOffsetParams>()
{
  return controller::msg::builder::Init_TcpOffsetParams_type();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__TCP_OFFSET_PARAMS__BUILDER_HPP_
