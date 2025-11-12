// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/MobileBaseCmd.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/mobile_base_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const MobileBaseCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: linear_velocity
  {
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << ", ";
  }

  // member: lifetime
  {
    out << "lifetime: ";
    rosidl_generator_traits::value_to_yaml(msg.lifetime, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MobileBaseCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << "\n";
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << "\n";
  }

  // member: lifetime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lifetime: ";
    rosidl_generator_traits::value_to_yaml(msg.lifetime, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MobileBaseCmd & msg, bool use_flow_style = false)
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
  const controller::msg::MobileBaseCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::MobileBaseCmd & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::MobileBaseCmd>()
{
  return "controller::msg::MobileBaseCmd";
}

template<>
inline const char * name<controller::msg::MobileBaseCmd>()
{
  return "controller/msg/MobileBaseCmd";
}

template<>
struct has_fixed_size<controller::msg::MobileBaseCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::msg::MobileBaseCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::msg::MobileBaseCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__TRAITS_HPP_
