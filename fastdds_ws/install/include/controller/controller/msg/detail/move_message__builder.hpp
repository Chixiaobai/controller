// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/MoveMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/move_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_MoveMessage_token
{
public:
  explicit Init_MoveMessage_token(::controller::msg::MoveMessage & msg)
  : msg_(msg)
  {}
  ::controller::msg::MoveMessage token(::controller::msg::MoveMessage::_token_type arg)
  {
    msg_.token = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::MoveMessage msg_;
};

class Init_MoveMessage_tcp_pose
{
public:
  explicit Init_MoveMessage_tcp_pose(::controller::msg::MoveMessage & msg)
  : msg_(msg)
  {}
  Init_MoveMessage_token tcp_pose(::controller::msg::MoveMessage::_tcp_pose_type arg)
  {
    msg_.tcp_pose = std::move(arg);
    return Init_MoveMessage_token(msg_);
  }

private:
  ::controller::msg::MoveMessage msg_;
};

class Init_MoveMessage_velocity
{
public:
  explicit Init_MoveMessage_velocity(::controller::msg::MoveMessage & msg)
  : msg_(msg)
  {}
  Init_MoveMessage_tcp_pose velocity(::controller::msg::MoveMessage::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_MoveMessage_tcp_pose(msg_);
  }

private:
  ::controller::msg::MoveMessage msg_;
};

class Init_MoveMessage_position
{
public:
  explicit Init_MoveMessage_position(::controller::msg::MoveMessage & msg)
  : msg_(msg)
  {}
  Init_MoveMessage_velocity position(::controller::msg::MoveMessage::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_MoveMessage_velocity(msg_);
  }

private:
  ::controller::msg::MoveMessage msg_;
};

class Init_MoveMessage_state
{
public:
  Init_MoveMessage_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveMessage_position state(::controller::msg::MoveMessage::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_MoveMessage_position(msg_);
  }

private:
  ::controller::msg::MoveMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::MoveMessage>()
{
  return controller::msg::builder::Init_MoveMessage_state();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__BUILDER_HPP_
