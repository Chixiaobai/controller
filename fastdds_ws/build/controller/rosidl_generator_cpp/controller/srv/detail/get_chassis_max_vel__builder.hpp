// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/GetChassisMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_CHASSIS_MAX_VEL__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__GET_CHASSIS_MAX_VEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/get_chassis_max_vel__struct.hpp"
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
auto build<::controller::srv::GetChassisMaxVel_Request>()
{
  return ::controller::srv::GetChassisMaxVel_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_GetChassisMaxVel_Response_angular_velocity
{
public:
  explicit Init_GetChassisMaxVel_Response_angular_velocity(::controller::srv::GetChassisMaxVel_Response & msg)
  : msg_(msg)
  {}
  ::controller::srv::GetChassisMaxVel_Response angular_velocity(::controller::srv::GetChassisMaxVel_Response::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::GetChassisMaxVel_Response msg_;
};

class Init_GetChassisMaxVel_Response_linear_velocity
{
public:
  Init_GetChassisMaxVel_Response_linear_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetChassisMaxVel_Response_angular_velocity linear_velocity(::controller::srv::GetChassisMaxVel_Response::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_GetChassisMaxVel_Response_angular_velocity(msg_);
  }

private:
  ::controller::srv::GetChassisMaxVel_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::GetChassisMaxVel_Response>()
{
  return controller::srv::builder::Init_GetChassisMaxVel_Response_linear_velocity();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__GET_CHASSIS_MAX_VEL__BUILDER_HPP_
