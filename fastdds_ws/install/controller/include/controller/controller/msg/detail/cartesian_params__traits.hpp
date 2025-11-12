// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/CartesianParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/cartesian_params__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const CartesianParams & msg,
  std::ostream & out)
{
  out << "{";
  // member: cartesian_index
  {
    out << "cartesian_index: ";
    rosidl_generator_traits::value_to_yaml(msg.cartesian_index, out);
    out << ", ";
  }

  // member: trans_max_vel
  {
    out << "trans_max_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.trans_max_vel, out);
    out << ", ";
  }

  // member: trans_max_acc
  {
    out << "trans_max_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.trans_max_acc, out);
    out << ", ";
  }

  // member: rota_max_vel
  {
    out << "rota_max_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.rota_max_vel, out);
    out << ", ";
  }

  // member: rota_max_acc
  {
    out << "rota_max_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.rota_max_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CartesianParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cartesian_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cartesian_index: ";
    rosidl_generator_traits::value_to_yaml(msg.cartesian_index, out);
    out << "\n";
  }

  // member: trans_max_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trans_max_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.trans_max_vel, out);
    out << "\n";
  }

  // member: trans_max_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trans_max_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.trans_max_acc, out);
    out << "\n";
  }

  // member: rota_max_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rota_max_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.rota_max_vel, out);
    out << "\n";
  }

  // member: rota_max_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rota_max_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.rota_max_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CartesianParams & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace controller

namespace rosidl_generator_traits
{

[[deprecated("use controller::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller::msg::CartesianParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::CartesianParams & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::CartesianParams>()
{
  return "controller::msg::CartesianParams";
}

template<>
inline const char * name<controller::msg::CartesianParams>()
{
  return "controller/msg/CartesianParams";
}

template<>
struct has_fixed_size<controller::msg::CartesianParams>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::msg::CartesianParams>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::msg::CartesianParams>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__TRAITS_HPP_
