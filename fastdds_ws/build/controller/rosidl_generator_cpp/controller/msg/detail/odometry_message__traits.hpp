// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/OdometryMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/odometry_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const OdometryMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: accumulated_time
  {
    out << "accumulated_time: ";
    rosidl_generator_traits::value_to_yaml(msg.accumulated_time, out);
    out << ", ";
  }

  // member: accumulated_mileage
  {
    out << "accumulated_mileage: ";
    rosidl_generator_traits::value_to_yaml(msg.accumulated_mileage, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const OdometryMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: accumulated_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accumulated_time: ";
    rosidl_generator_traits::value_to_yaml(msg.accumulated_time, out);
    out << "\n";
  }

  // member: accumulated_mileage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accumulated_mileage: ";
    rosidl_generator_traits::value_to_yaml(msg.accumulated_mileage, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const OdometryMessage & msg, bool use_flow_style = false)
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
  const controller::msg::OdometryMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::OdometryMessage & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::OdometryMessage>()
{
  return "controller::msg::OdometryMessage";
}

template<>
inline const char * name<controller::msg::OdometryMessage>()
{
  return "controller/msg/OdometryMessage";
}

template<>
struct has_fixed_size<controller::msg::OdometryMessage>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::msg::OdometryMessage>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::msg::OdometryMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__TRAITS_HPP_
