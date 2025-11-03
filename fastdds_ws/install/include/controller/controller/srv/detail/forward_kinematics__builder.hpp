// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/ForwardKinematics.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/forward_kinematics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_ForwardKinematics_Request_joint_angles
{
public:
  explicit Init_ForwardKinematics_Request_joint_angles(::controller::srv::ForwardKinematics_Request & msg)
  : msg_(msg)
  {}
  ::controller::srv::ForwardKinematics_Request joint_angles(::controller::srv::ForwardKinematics_Request::_joint_angles_type arg)
  {
    msg_.joint_angles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::ForwardKinematics_Request msg_;
};

class Init_ForwardKinematics_Request_type
{
public:
  Init_ForwardKinematics_Request_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ForwardKinematics_Request_joint_angles type(::controller::srv::ForwardKinematics_Request::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_ForwardKinematics_Request_joint_angles(msg_);
  }

private:
  ::controller::srv::ForwardKinematics_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::ForwardKinematics_Request>()
{
  return controller::srv::builder::Init_ForwardKinematics_Request_type();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_ForwardKinematics_Response_pose
{
public:
  Init_ForwardKinematics_Response_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::ForwardKinematics_Response pose(::controller::srv::ForwardKinematics_Response::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::ForwardKinematics_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::ForwardKinematics_Response>()
{
  return controller::srv::builder::Init_ForwardKinematics_Response_pose();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__BUILDER_HPP_
