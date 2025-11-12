// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/SetChassisMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/set_chassis_max_vel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetChassisMaxVel_Request_angular_velocity
{
public:
  explicit Init_SetChassisMaxVel_Request_angular_velocity(::controller::srv::SetChassisMaxVel_Request & msg)
  : msg_(msg)
  {}
  ::controller::srv::SetChassisMaxVel_Request angular_velocity(::controller::srv::SetChassisMaxVel_Request::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetChassisMaxVel_Request msg_;
};

class Init_SetChassisMaxVel_Request_linear_velocity
{
public:
  Init_SetChassisMaxVel_Request_linear_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetChassisMaxVel_Request_angular_velocity linear_velocity(::controller::srv::SetChassisMaxVel_Request::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_SetChassisMaxVel_Request_angular_velocity(msg_);
  }

private:
  ::controller::srv::SetChassisMaxVel_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetChassisMaxVel_Request>()
{
  return controller::srv::builder::Init_SetChassisMaxVel_Request_linear_velocity();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_SetChassisMaxVel_Response_success
{
public:
  Init_SetChassisMaxVel_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::SetChassisMaxVel_Response success(::controller::srv::SetChassisMaxVel_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::SetChassisMaxVel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::SetChassisMaxVel_Response>()
{
  return controller::srv::builder::Init_SetChassisMaxVel_Response_success();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__BUILDER_HPP_
