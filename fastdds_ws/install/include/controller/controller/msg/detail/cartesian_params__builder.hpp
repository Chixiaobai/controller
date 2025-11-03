// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from controller:msg/CartesianParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__BUILDER_HPP_
#define CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "controller/msg/detail/cartesian_params__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace controller
{

namespace msg
{

namespace builder
{

class Init_CartesianParams_rota_max_acc
{
public:
  explicit Init_CartesianParams_rota_max_acc(::controller::msg::CartesianParams & msg)
  : msg_(msg)
  {}
  ::controller::msg::CartesianParams rota_max_acc(::controller::msg::CartesianParams::_rota_max_acc_type arg)
  {
    msg_.rota_max_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::controller::msg::CartesianParams msg_;
};

class Init_CartesianParams_rota_max_vel
{
public:
  explicit Init_CartesianParams_rota_max_vel(::controller::msg::CartesianParams & msg)
  : msg_(msg)
  {}
  Init_CartesianParams_rota_max_acc rota_max_vel(::controller::msg::CartesianParams::_rota_max_vel_type arg)
  {
    msg_.rota_max_vel = std::move(arg);
    return Init_CartesianParams_rota_max_acc(msg_);
  }

private:
  ::controller::msg::CartesianParams msg_;
};

class Init_CartesianParams_trans_max_acc
{
public:
  explicit Init_CartesianParams_trans_max_acc(::controller::msg::CartesianParams & msg)
  : msg_(msg)
  {}
  Init_CartesianParams_rota_max_vel trans_max_acc(::controller::msg::CartesianParams::_trans_max_acc_type arg)
  {
    msg_.trans_max_acc = std::move(arg);
    return Init_CartesianParams_rota_max_vel(msg_);
  }

private:
  ::controller::msg::CartesianParams msg_;
};

class Init_CartesianParams_trans_max_vel
{
public:
  explicit Init_CartesianParams_trans_max_vel(::controller::msg::CartesianParams & msg)
  : msg_(msg)
  {}
  Init_CartesianParams_trans_max_acc trans_max_vel(::controller::msg::CartesianParams::_trans_max_vel_type arg)
  {
    msg_.trans_max_vel = std::move(arg);
    return Init_CartesianParams_trans_max_acc(msg_);
  }

private:
  ::controller::msg::CartesianParams msg_;
};

class Init_CartesianParams_cartesian_index
{
public:
  Init_CartesianParams_cartesian_index()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianParams_trans_max_vel cartesian_index(::controller::msg::CartesianParams::_cartesian_index_type arg)
  {
    msg_.cartesian_index = std::move(arg);
    return Init_CartesianParams_trans_max_vel(msg_);
  }

private:
  ::controller::msg::CartesianParams msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::controller::msg::CartesianParams>()
{
  return controller::msg::builder::Init_CartesianParams_cartesian_index();
}

}  // namespace controller

#endif  // CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__BUILDER_HPP_
