// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/MoveMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/move_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'tcp_pose'
#include "controller/msg/detail/tcp_pose_params__traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const MoveMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: velocity
  {
    if (msg.velocity.size() == 0) {
      out << "velocity: []";
    } else {
      out << "velocity: [";
      size_t pending_items = msg.velocity.size();
      for (auto item : msg.velocity) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tcp_pose
  {
    if (msg.tcp_pose.size() == 0) {
      out << "tcp_pose: []";
    } else {
      out << "tcp_pose: [";
      size_t pending_items = msg.tcp_pose.size();
      for (auto item : msg.tcp_pose) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: token
  {
    out << "token: ";
    rosidl_generator_traits::value_to_yaml(msg.token, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.velocity.size() == 0) {
      out << "velocity: []\n";
    } else {
      out << "velocity:\n";
      for (auto item : msg.velocity) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tcp_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tcp_pose.size() == 0) {
      out << "tcp_pose: []\n";
    } else {
      out << "tcp_pose:\n";
      for (auto item : msg.tcp_pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: token
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "token: ";
    rosidl_generator_traits::value_to_yaml(msg.token, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveMessage & msg, bool use_flow_style = false)
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
  const controller::msg::MoveMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::MoveMessage & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::MoveMessage>()
{
  return "controller::msg::MoveMessage";
}

template<>
inline const char * name<controller::msg::MoveMessage>()
{
  return "controller/msg/MoveMessage";
}

template<>
struct has_fixed_size<controller::msg::MoveMessage>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::msg::MoveMessage>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::msg::MoveMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__TRAITS_HPP_
