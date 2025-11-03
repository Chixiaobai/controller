// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/JointParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__JOINT_PARAMS__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__JOINT_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/joint_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_JointParams_max_acc
{
public:
  explicit Init_JointParams_max_acc(::controller::msg::JointParams & msg)
  : msg_(msg)
  {}
  ::controller::msg::JointParams max_acc(::controller::msg::JointParams::_max_acc_type arg)
  {
    msg_.max_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::JointParams msg_;
};

class Init_JointParams_max_vel
{
public:
  explicit Init_JointParams_max_vel(::controller::msg::JointParams & msg)
  : msg_(msg)
  {}
  Init_JointParams_max_acc max_vel(::controller::msg::JointParams::_max_vel_type arg)
  {
    msg_.max_vel = std::move(arg);
    return Init_JointParams_max_acc(msg_);
  }

private:
  ::controller::msg::JointParams msg_;
};

class Init_JointParams_min_pos
{
public:
  explicit Init_JointParams_min_pos(::controller::msg::JointParams & msg)
  : msg_(msg)
  {}
  Init_JointParams_max_vel min_pos(::controller::msg::JointParams::_min_pos_type arg)
  {
    msg_.min_pos = std::move(arg);
    return Init_JointParams_max_vel(msg_);
  }

private:
  ::controller::msg::JointParams msg_;
};

class Init_JointParams_max_pos
{
public:
  explicit Init_JointParams_max_pos(::controller::msg::JointParams & msg)
  : msg_(msg)
  {}
  Init_JointParams_min_pos max_pos(::controller::msg::JointParams::_max_pos_type arg)
  {
    msg_.max_pos = std::move(arg);
    return Init_JointParams_min_pos(msg_);
  }

private:
  ::controller::msg::JointParams msg_;
};

class Init_JointParams_joint_index
{
public:
  Init_JointParams_joint_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointParams_max_pos joint_index(::controller::msg::JointParams::_joint_index_type arg)
  {
    msg_.joint_index = std::move(arg);
    return Init_JointParams_max_pos(msg_);
  }

private:
  ::controller::msg::JointParams msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::JointParams>()
{
  return controller::msg::builder::Init_JointParams_joint_index();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__JOINT_PARAMS__BUILDER_HPP_
