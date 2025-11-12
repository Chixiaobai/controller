// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/LinearMoveParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/linear_move_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_LinearMoveParams_acceleration_percent
{
public:
  explicit Init_LinearMoveParams_acceleration_percent(::controller::msg::LinearMoveParams & msg)
  : msg_(msg)
  {}
  ::controller::msg::LinearMoveParams acceleration_percent(::controller::msg::LinearMoveParams::_acceleration_percent_type arg)
  {
    msg_.acceleration_percent = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::LinearMoveParams msg_;
};

class Init_LinearMoveParams_velocity_percent
{
public:
  explicit Init_LinearMoveParams_velocity_percent(::controller::msg::LinearMoveParams & msg)
  : msg_(msg)
  {}
  Init_LinearMoveParams_acceleration_percent velocity_percent(::controller::msg::LinearMoveParams::_velocity_percent_type arg)
  {
    msg_.velocity_percent = std::move(arg);
    return Init_LinearMoveParams_acceleration_percent(msg_);
  }

private:
  ::controller::msg::LinearMoveParams msg_;
};

class Init_LinearMoveParams_pose
{
public:
  explicit Init_LinearMoveParams_pose(::controller::msg::LinearMoveParams & msg)
  : msg_(msg)
  {}
  Init_LinearMoveParams_velocity_percent pose(::controller::msg::LinearMoveParams::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_LinearMoveParams_velocity_percent(msg_);
  }

private:
  ::controller::msg::LinearMoveParams msg_;
};

class Init_LinearMoveParams_type
{
public:
  Init_LinearMoveParams_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LinearMoveParams_pose type(::controller::msg::LinearMoveParams::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_LinearMoveParams_pose(msg_);
  }

private:
  ::controller::msg::LinearMoveParams msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::LinearMoveParams>()
{
  return controller::msg::builder::Init_LinearMoveParams_type();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__BUILDER_HPP_
