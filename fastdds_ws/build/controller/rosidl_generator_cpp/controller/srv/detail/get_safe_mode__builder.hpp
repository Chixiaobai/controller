// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetSafeMode.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_SAFE_MODE__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_SAFE_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_safe_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetSafeMode_Request>()
{
  return ::controller::srv::GetSafeMode_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetSafeMode_Response_enable
{
public:
  Init_GetSafeMode_Response_enable()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::GetSafeMode_Response enable(::controller::srv::GetSafeMode_Response::_enable_type arg)
  {
    msg_.enable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetSafeMode_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetSafeMode_Response>()
{
  return controller::srv::builder::Init_GetSafeMode_Response_enable();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_SAFE_MODE__BUILDER_HPP_
