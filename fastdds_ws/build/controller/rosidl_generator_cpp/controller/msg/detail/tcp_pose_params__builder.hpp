// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/TcpPoseParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/tcp_pose_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_TcpPoseParams_pose
{
public:
  explicit Init_TcpPoseParams_pose(::controller::msg::TcpPoseParams & msg)
  : msg_(msg)
  {}
  ::controller::msg::TcpPoseParams pose(::controller::msg::TcpPoseParams::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::TcpPoseParams msg_;
};

class Init_TcpPoseParams_type
{
public:
  Init_TcpPoseParams_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TcpPoseParams_pose type(::controller::msg::TcpPoseParams::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_TcpPoseParams_pose(msg_);
  }

private:
  ::controller::msg::TcpPoseParams msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::TcpPoseParams>()
{
  return controller::msg::builder::Init_TcpPoseParams_type();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__BUILDER_HPP_
