// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/JointParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__JOINT_PARAMS__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__JOINT_PARAMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/joint_params__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointParams & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_index
  {
    out << "joint_index: ";
    rosidl_generator_traits::value_to_yaml(msg.joint_index, out);
    out << ", ";
  }

  // member: max_pos
  {
    out << "max_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.max_pos, out);
    out << ", ";
  }

  // member: min_pos
  {
    out << "min_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.min_pos, out);
    out << ", ";
  }

  // member: max_vel
  {
    out << "max_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.max_vel, out);
    out << ", ";
  }

  // member: max_acc
  {
    out << "max_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointParams & msg,
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

  // member: max_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.max_pos, out);
    out << "\n";
  }

  // member: min_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.min_pos, out);
    out << "\n";
  }

  // member: max_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.max_vel, out);
    out << "\n";
  }

  // member: max_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointParams & msg, bool use_flow_style = false)
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
  const controller::msg::JointParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::JointParams & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::JointParams>()
{
  return "controller::msg::JointParams";
}

template<>
inline const char * name<controller::msg::JointParams>()
{
  return "controller/msg/JointParams";
}

template<>
struct has_fixed_size<controller::msg::JointParams>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::msg::JointParams>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::msg::JointParams>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__JOINT_PARAMS__TRAITS_HPP_
