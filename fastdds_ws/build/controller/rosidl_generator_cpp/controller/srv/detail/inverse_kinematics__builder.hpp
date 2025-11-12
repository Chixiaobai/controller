// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__BUILDER_HPP_
#define CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/srv/detail/inverse_kinematics__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace srv
{

namespace builder
{

class Init_InverseKinematics_Request_if_use_whole_body
{
public:
  explicit Init_InverseKinematics_Request_if_use_whole_body(::controller::srv::InverseKinematics_Request & msg)
  : msg_(msg)
  {}
  ::controller::srv::InverseKinematics_Request if_use_whole_body(::controller::srv::InverseKinematics_Request::_if_use_whole_body_type arg)
  {
    msg_.if_use_whole_body = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::InverseKinematics_Request msg_;
};

class Init_InverseKinematics_Request_reference_joint_angles
{
public:
  explicit Init_InverseKinematics_Request_reference_joint_angles(::controller::srv::InverseKinematics_Request & msg)
  : msg_(msg)
  {}
  Init_InverseKinematics_Request_if_use_whole_body reference_joint_angles(::controller::srv::InverseKinematics_Request::_reference_joint_angles_type arg)
  {
    msg_.reference_joint_angles = std::move(arg);
    return Init_InverseKinematics_Request_if_use_whole_body(msg_);
  }

private:
  ::controller::srv::InverseKinematics_Request msg_;
};

class Init_InverseKinematics_Request_pose
{
public:
  Init_InverseKinematics_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_InverseKinematics_Request_reference_joint_angles pose(::controller::srv::InverseKinematics_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_InverseKinematics_Request_reference_joint_angles(msg_);
  }

private:
  ::controller::srv::InverseKinematics_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::InverseKinematics_Request>()
{
  return controller::srv::builder::Init_InverseKinematics_Request_pose();
}

}  // namespace controller


namespace controller
{

namespace srv
{

namespace builder
{

class Init_InverseKinematics_Response_joint_angles
{
public:
  Init_InverseKinematics_Response_joint_angles()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::controller::srv::InverseKinematics_Response joint_angles(::controller::srv::InverseKinematics_Response::_joint_angles_type arg)
  {
    msg_.joint_angles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::srv::InverseKinematics_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::srv::InverseKinematics_Response>()
{
  return controller::srv::builder::Init_InverseKinematics_Response_joint_angles();
}

}  // namespace controller

#endif  // CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__BUILDER_HPP_
