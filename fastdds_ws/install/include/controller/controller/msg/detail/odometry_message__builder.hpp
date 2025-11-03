// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/OdometryMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/odometry_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_OdometryMessage_accumulated_mileage
{
public:
  explicit Init_OdometryMessage_accumulated_mileage(::controller::msg::OdometryMessage & msg)
  : msg_(msg)
  {}
  ::controller::msg::OdometryMessage accumulated_mileage(::controller::msg::OdometryMessage::_accumulated_mileage_type arg)
  {
    msg_.accumulated_mileage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::OdometryMessage msg_;
};

class Init_OdometryMessage_accumulated_time
{
public:
  Init_OdometryMessage_accumulated_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OdometryMessage_accumulated_mileage accumulated_time(::controller::msg::OdometryMessage::_accumulated_time_type arg)
  {
    msg_.accumulated_time = std::move(arg);
    return Init_OdometryMessage_accumulated_mileage(msg_);
  }

private:
  ::controller::msg::OdometryMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::OdometryMessage>()
{
  return controller::msg::builder::Init_OdometryMessage_accumulated_time();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__BUILDER_HPP_
