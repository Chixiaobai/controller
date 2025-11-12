// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/ErrorMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/error_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_ErrorMessage_msg
{
public:
  explicit Init_ErrorMessage_msg(::controller::msg::ErrorMessage & msg)
  : msg_(msg)
  {}
  ::controller::msg::ErrorMessage msg(::controller::msg::ErrorMessage::_msg_type arg)
  {
    msg_.msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::ErrorMessage msg_;
};

class Init_ErrorMessage_error_code
{
public:
  explicit Init_ErrorMessage_error_code(::controller::msg::ErrorMessage & msg)
  : msg_(msg)
  {}
  Init_ErrorMessage_msg error_code(::controller::msg::ErrorMessage::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_ErrorMessage_msg(msg_);
  }

private:
  ::controller::msg::ErrorMessage msg_;
};

class Init_ErrorMessage_module
{
public:
  explicit Init_ErrorMessage_module(::controller::msg::ErrorMessage & msg)
  : msg_(msg)
  {}
  Init_ErrorMessage_error_code module(::controller::msg::ErrorMessage::_module_type arg)
  {
    msg_.module = std::move(arg);
    return Init_ErrorMessage_error_code(msg_);
  }

private:
  ::controller::msg::ErrorMessage msg_;
};

class Init_ErrorMessage_level
{
public:
  explicit Init_ErrorMessage_level(::controller::msg::ErrorMessage & msg)
  : msg_(msg)
  {}
  Init_ErrorMessage_module level(::controller::msg::ErrorMessage::_level_type arg)
  {
    msg_.level = std::move(arg);
    return Init_ErrorMessage_module(msg_);
  }

private:
  ::controller::msg::ErrorMessage msg_;
};

class Init_ErrorMessage_id
{
public:
  Init_ErrorMessage_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ErrorMessage_level id(::controller::msg::ErrorMessage::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_ErrorMessage_level(msg_);
  }

private:
  ::controller::msg::ErrorMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::ErrorMessage>()
{
  return controller::msg::builder::Init_ErrorMessage_id();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__BUILDER_HPP_
