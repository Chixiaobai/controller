// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/RealTimeBodyTcpCartesian.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/real_time_body_tcp_cartesian__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_RealTimeBodyTcpCartesian_time
{
public:
  explicit Init_RealTimeBodyTcpCartesian_time(::controller::msg::RealTimeBodyTcpCartesian & msg)
  : msg_(msg)
  {}
  ::controller::msg::RealTimeBodyTcpCartesian time(::controller::msg::RealTimeBodyTcpCartesian::_time_type arg)
  {
    msg_.time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::RealTimeBodyTcpCartesian msg_;
};

class Init_RealTimeBodyTcpCartesian_torso_valid
{
public:
  explicit Init_RealTimeBodyTcpCartesian_torso_valid(::controller::msg::RealTimeBodyTcpCartesian & msg)
  : msg_(msg)
  {}
  Init_RealTimeBodyTcpCartesian_time torso_valid(::controller::msg::RealTimeBodyTcpCartesian::_torso_valid_type arg)
  {
    msg_.torso_valid = std::move(arg);
    return Init_RealTimeBodyTcpCartesian_time(msg_);
  }

private:
  ::controller::msg::RealTimeBodyTcpCartesian msg_;
};

class Init_RealTimeBodyTcpCartesian_torso
{
public:
  explicit Init_RealTimeBodyTcpCartesian_torso(::controller::msg::RealTimeBodyTcpCartesian & msg)
  : msg_(msg)
  {}
  Init_RealTimeBodyTcpCartesian_torso_valid torso(::controller::msg::RealTimeBodyTcpCartesian::_torso_type arg)
  {
    msg_.torso = std::move(arg);
    return Init_RealTimeBodyTcpCartesian_torso_valid(msg_);
  }

private:
  ::controller::msg::RealTimeBodyTcpCartesian msg_;
};

class Init_RealTimeBodyTcpCartesian_right_arm_valid
{
public:
  explicit Init_RealTimeBodyTcpCartesian_right_arm_valid(::controller::msg::RealTimeBodyTcpCartesian & msg)
  : msg_(msg)
  {}
  Init_RealTimeBodyTcpCartesian_torso right_arm_valid(::controller::msg::RealTimeBodyTcpCartesian::_right_arm_valid_type arg)
  {
    msg_.right_arm_valid = std::move(arg);
    return Init_RealTimeBodyTcpCartesian_torso(msg_);
  }

private:
  ::controller::msg::RealTimeBodyTcpCartesian msg_;
};

class Init_RealTimeBodyTcpCartesian_right_arm
{
public:
  explicit Init_RealTimeBodyTcpCartesian_right_arm(::controller::msg::RealTimeBodyTcpCartesian & msg)
  : msg_(msg)
  {}
  Init_RealTimeBodyTcpCartesian_right_arm_valid right_arm(::controller::msg::RealTimeBodyTcpCartesian::_right_arm_type arg)
  {
    msg_.right_arm = std::move(arg);
    return Init_RealTimeBodyTcpCartesian_right_arm_valid(msg_);
  }

private:
  ::controller::msg::RealTimeBodyTcpCartesian msg_;
};

class Init_RealTimeBodyTcpCartesian_left_arm_valid
{
public:
  explicit Init_RealTimeBodyTcpCartesian_left_arm_valid(::controller::msg::RealTimeBodyTcpCartesian & msg)
  : msg_(msg)
  {}
  Init_RealTimeBodyTcpCartesian_right_arm left_arm_valid(::controller::msg::RealTimeBodyTcpCartesian::_left_arm_valid_type arg)
  {
    msg_.left_arm_valid = std::move(arg);
    return Init_RealTimeBodyTcpCartesian_right_arm(msg_);
  }

private:
  ::controller::msg::RealTimeBodyTcpCartesian msg_;
};

class Init_RealTimeBodyTcpCartesian_left_arm
{
public:
  explicit Init_RealTimeBodyTcpCartesian_left_arm(::controller::msg::RealTimeBodyTcpCartesian & msg)
  : msg_(msg)
  {}
  Init_RealTimeBodyTcpCartesian_left_arm_valid left_arm(::controller::msg::RealTimeBodyTcpCartesian::_left_arm_type arg)
  {
    msg_.left_arm = std::move(arg);
    return Init_RealTimeBodyTcpCartesian_left_arm_valid(msg_);
  }

private:
  ::controller::msg::RealTimeBodyTcpCartesian msg_;
};

class Init_RealTimeBodyTcpCartesian_id
{
public:
  Init_RealTimeBodyTcpCartesian_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RealTimeBodyTcpCartesian_left_arm id(::controller::msg::RealTimeBodyTcpCartesian::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_RealTimeBodyTcpCartesian_left_arm(msg_);
  }

private:
  ::controller::msg::RealTimeBodyTcpCartesian msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::RealTimeBodyTcpCartesian>()
{
  return controller::msg::builder::Init_RealTimeBodyTcpCartesian_id();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__BUILDER_HPP_
