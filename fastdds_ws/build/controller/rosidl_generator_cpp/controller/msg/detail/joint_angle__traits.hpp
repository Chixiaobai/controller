// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/JointAngle.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__JOINT_ANGLE__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__JOINT_ANGLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/joint_angle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointAngle & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_index
  {
    out << "joint_index: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_index, out);
    out << ", ";
  }

  // member: target_position
  {
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_index: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_index, out);
    out << "\n";
  }

  // member: target_position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_position: ";
    rosidl_generator_traits::value_to_yaml(msg.target_position, out);
    out << "\n";
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointAngle & msg, bool use_flow_style = false)
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
  const controller::msg::JointAngle & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::JointAngle & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::JointAngle>()
{
  return "controller::msg::JointAngle";
}

template<>
inline const char * name<controller::msg::JointAngle>()
{
  return "controller/msg/JointAngle";
}

template<>
struct has_fixed_size<controller::msg::JointAngle>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::msg::JointAngle>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::msg::JointAngle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__JOINT_ANGLE__TRAITS_HPP_
