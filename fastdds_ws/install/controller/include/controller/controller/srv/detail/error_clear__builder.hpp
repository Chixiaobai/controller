// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/ErrorClear.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__ERROR_CLEAR__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__ERROR_CLEAR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/error_clear__struct.hpp"
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
auto build<::controller::srv::ErrorClear_Request>()
{
  return ::controller::srv::ErrorClear_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_ErrorClear_Response_success
{
public:
  Init_ErrorClear_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::ErrorClear_Response success(::controller::srv::ErrorClear_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::ErrorClear_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::ErrorClear_Response>()
{
  return controller::srv::builder::Init_ErrorClear_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__ERROR_CLEAR__BUILDER_HPP_
