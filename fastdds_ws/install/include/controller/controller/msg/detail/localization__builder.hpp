// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/Localization.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__LOCALIZATION__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__LOCALIZATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/localization__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_Localization_trajectory_id
{
public:
  explicit Init_Localization_trajectory_id(::controller::msg::Localization & msg)
  : msg_(msg)
  {}
  ::controller::msg::Localization trajectory_id(::controller::msg::Localization::_trajectory_id_type arg)
  {
    msg_.trajectory_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::Localization msg_;
};

class Init_Localization_error_msgs
{
public:
  explicit Init_Localization_error_msgs(::controller::msg::Localization & msg)
  : msg_(msg)
  {}
  Init_Localization_trajectory_id error_msgs(::controller::msg::Localization::_error_msgs_type arg)
  {
    msg_.error_msgs = std::move(arg);
    return Init_Localization_trajectory_id(msg_);
  }

private:
  ::controller::msg::Localization msg_;
};

class Init_Localization_valid_flag
{
public:
  explicit Init_Localization_valid_flag(::controller::msg::Localization & msg)
  : msg_(msg)
  {}
  Init_Localization_error_msgs valid_flag(::controller::msg::Localization::_valid_flag_type arg)
  {
    msg_.valid_flag = std::move(arg);
    return Init_Localization_error_msgs(msg_);
  }

private:
  ::controller::msg::Localization msg_;
};

class Init_Localization_confidence
{
public:
  explicit Init_Localization_confidence(::controller::msg::Localization & msg)
  : msg_(msg)
  {}
  Init_Localization_valid_flag confidence(::controller::msg::Localization::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Localization_valid_flag(msg_);
  }

private:
  ::controller::msg::Localization msg_;
};

class Init_Localization_pose
{
public:
  explicit Init_Localization_pose(::controller::msg::Localization & msg)
  : msg_(msg)
  {}
  Init_Localization_confidence pose(::controller::msg::Localization::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Localization_confidence(msg_);
  }

private:
  ::controller::msg::Localization msg_;
};

class Init_Localization_header
{
public:
  Init_Localization_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Localization_pose header(::controller::msg::Localization::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Localization_pose(msg_);
  }

private:
  ::controller::msg::Localization msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::Localization>()
{
  return controller::msg::builder::Init_Localization_header();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__LOCALIZATION__BUILDER_HPP_
