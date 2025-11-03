// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:msg/RealTimeBodyTcpCartesian.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__TRAITS_HPP_
#define CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/msg/detail/real_time_body_tcp_cartesian__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace msg
{

inline void to_flow_style_yaml(
  const RealTimeBodyTcpCartesian & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: left_arm
  {
    if (msg.left_arm.size() == 0) {
      out << "left_arm: []";
    } else {
      out << "left_arm: [";
      size_t pending_items = msg.left_arm.size();
      for (auto item : msg.left_arm) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: left_arm_valid
  {
    out << "left_arm_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.left_arm_valid, out);
    out << ", ";
  }

  // member: right_arm
  {
    if (msg.right_arm.size() == 0) {
      out << "right_arm: []";
    } else {
      out << "right_arm: [";
      size_t pending_items = msg.right_arm.size();
      for (auto item : msg.right_arm) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: right_arm_valid
  {
    out << "right_arm_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.right_arm_valid, out);
    out << ", ";
  }

  // member: torso
  {
    if (msg.torso.size() == 0) {
      out << "torso: []";
    } else {
      out << "torso: [";
      size_t pending_items = msg.torso.size();
      for (auto item : msg.torso) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: torso_valid
  {
    out << "torso_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_valid, out);
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RealTimeBodyTcpCartesian & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: left_arm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.left_arm.size() == 0) {
      out << "left_arm: []\n";
    } else {
      out << "left_arm:\n";
      for (auto item : msg.left_arm) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: left_arm_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_arm_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.left_arm_valid, out);
    out << "\n";
  }

  // member: right_arm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.right_arm.size() == 0) {
      out << "right_arm: []\n";
    } else {
      out << "right_arm:\n";
      for (auto item : msg.right_arm) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: right_arm_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_arm_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.right_arm_valid, out);
    out << "\n";
  }

  // member: torso
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.torso.size() == 0) {
      out << "torso: []\n";
    } else {
      out << "torso:\n";
      for (auto item : msg.torso) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: torso_valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torso_valid: ";
    rosidl_generator_traits::value_to_yaml(msg.torso_valid, out);
    out << "\n";
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RealTimeBodyTcpCartesian & msg, bool use_flow_style = false)
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
  const controller::msg::RealTimeBodyTcpCartesian & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::msg::to_yaml() instead")]]
inline std::string to_yaml(const controller::msg::RealTimeBodyTcpCartesian & msg)
{
  return controller::msg::to_yaml(msg);
}

template<>
inline const char * data_type<controller::msg::RealTimeBodyTcpCartesian>()
{
  return "controller::msg::RealTimeBodyTcpCartesian";
}

template<>
inline const char * name<controller::msg::RealTimeBodyTcpCartesian>()
{
  return "controller/msg/RealTimeBodyTcpCartesian";
}

template<>
struct has_fixed_size<controller::msg::RealTimeBodyTcpCartesian>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::msg::RealTimeBodyTcpCartesian>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::msg::RealTimeBodyTcpCartesian>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__TRAITS_HPP_
