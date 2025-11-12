// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/LinearMoveParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/linear_move_params__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const LinearMoveParams & msg,
  std::ostream & out)
{
  out << "{";
  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: pose
  {
    if (msg.pose.size() == 0) {
      out << "pose: []";
    } else {
      out << "pose: [";
      size_t pending_items = msg.pose.size();
      for (auto item : msg.pose) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocity_percent
  {
    out << "velocity_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_percent, out);
    out << ", ";
  }

  // member: acceleration_percent
  {
    out << "acceleration_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration_percent, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LinearMoveParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose.size() == 0) {
      out << "pose: []\n";
    } else {
      out << "pose:\n";
      for (auto item : msg.pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: velocity_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity_percent, out);
    out << "\n";
  }

  // member: acceleration_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.acceleration_percent, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LinearMoveParams & msg, bool use_flow_style = false)
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
  const controller::msg::LinearMoveParams & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::LinearMoveParams & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::LinearMoveParams>()
{
  return "controller::msg::LinearMoveParams";
}

template<>
inline const char * name<controller::msg::LinearMoveParams>()
{
  return "controller/msg/LinearMoveParams";
}

template<>
struct has_fixed_size<controller::msg::LinearMoveParams>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::msg::LinearMoveParams>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::msg::LinearMoveParams>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__LINEAR_MOVE_PARAMS__TRAITS_HPP_
