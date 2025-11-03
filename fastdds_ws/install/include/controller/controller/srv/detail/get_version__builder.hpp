// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetVersion.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_VERSION__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_VERSION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_version__struct.hpp"
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
auto build<::controller::srv::GetVersion_Request>()
{
  return ::controller::srv::GetVersion_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetVersion_Response_plugin_versions
{
public:
  explicit Init_GetVersion_Response_plugin_versions(::controller::srv::GetVersion_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::GetVersion_Response plugin_versions(::controller::srv::GetVersion_Response::_plugin_versions_type arg)
  {
    msg_.plugin_versions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetVersion_Response msg_;
};

class Init_GetVersion_Response_plugin_names
{
public:
  explicit Init_GetVersion_Response_plugin_names(::controller::srv::GetVersion_Response & msg)
  : msg_(msg)
  {}
  Init_GetVersion_Response_plugin_versions plugin_names(::controller::srv::GetVersion_Response::_plugin_names_type arg)
  {
    msg_.plugin_names = std::move(arg);
    return Init_GetVersion_Response_plugin_versions(msg_);
  }

private:
  ::controller::srv::GetVersion_Response msg_;
};

class Init_GetVersion_Response_main
{
public:
  Init_GetVersion_Response_main()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetVersion_Response_plugin_names main(::controller::srv::GetVersion_Response::_main_type arg)
  {
    msg_.main = std::move(arg);
    return Init_GetVersion_Response_plugin_names(msg_);
  }

private:
  ::controller::srv::GetVersion_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetVersion_Response>()
{
  return controller::srv::builder::Init_GetVersion_Response_main();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_VERSION__BUILDER_HPP_
