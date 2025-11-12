// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/JointAngle.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/joint_angle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_JointAngle_velocity
{
public:
  explicit Init_JointAngle_velocity(::controller::msg::JointAngle & msg)
  : msg_(msg)
  {}
  ::controller::msg::JointAngle velocity(::controller::msg::JointAngle::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::JointAngle msg_;
};

class Init_JointAngle_target_position
{
public:
  explicit Init_JointAngle_target_position(::controller::msg::JointAngle & msg)
  : msg_(msg)
  {}
  Init_JointAngle_velocity target_position(::controller::msg::JointAngle::_target_position_type arg)
  {
    msg_.target_position = std::move(arg);
    return Init_JointAngle_velocity(msg_);
  }

private:
  ::controller::msg::JointAngle msg_;
};

class Init_JointAngle_joint_index
{
public:
  Init_JointAngle_joint_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointAngle_target_position joint_index(::controller::msg::JointAngle::_joint_index_type arg)
  {
    msg_.joint_index = std::move(arg);
    return Init_JointAngle_target_position(msg_);
  }

private:
  ::controller::msg::JointAngle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::JointAngle>()
{
  return controller::msg::builder::Init_JointAngle_joint_index();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__JOINT_ANGLE__BUILDER_HPP_
