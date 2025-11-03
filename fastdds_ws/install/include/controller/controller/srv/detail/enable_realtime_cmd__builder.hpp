// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/EnableRealtimeCmd.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__ENABLE_REALTIME_CMD__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__ENABLE_REALTIME_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/enable_realtime_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_EnableRealtimeCmd_Request_enable
{
public:
  Init_EnableRealtimeCmd_Request_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::EnableRealtimeCmd_Request enable(::controller::srv::EnableRealtimeCmd_Request::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::EnableRealtimeCmd_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::EnableRealtimeCmd_Request>()
{
  return controller::srv::builder::Init_EnableRealtimeCmd_Request_enable();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_EnableRealtimeCmd_Response_success
{
public:
  Init_EnableRealtimeCmd_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::EnableRealtimeCmd_Response success(::controller::srv::EnableRealtimeCmd_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::EnableRealtimeCmd_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::EnableRealtimeCmd_Response>()
{
  return controller::srv::builder::Init_EnableRealtimeCmd_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__ENABLE_REALTIME_CMD__BUILDER_HPP_
