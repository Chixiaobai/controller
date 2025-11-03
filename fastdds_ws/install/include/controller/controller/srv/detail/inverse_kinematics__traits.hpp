// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/inverse_kinematics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "controller/msg/detail/tcp_pose_params__traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const InverseKinematics_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: pose
  {
    if (msg.pose.size() == 0) {
      out << "pose: []";
    } else {
      out << "pose: [";
      size_t pending_items = msg.pose.size();
      for (auto item : msg.pose) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: if_use_whole_body
  {
    out << "if_use_whole_body: ";
    rosidl_generator_traits::value_to_yaml(msg.if_use_whole_body, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InverseKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
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
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: if_use_whole_body
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "if_use_whole_body: ";
    rosidl_generator_traits::value_to_yaml(msg.if_use_whole_body, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InverseKinematics_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace controller

namespace rosidl_generator_traits
{

[[deprecated("use controller::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller::srv::InverseKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::InverseKinematics_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::InverseKinematics_Request>()
{
  return "controller::srv::InverseKinematics_Request";
}

template<>
inline const char * name<controller::srv::InverseKinematics_Request>()
{
  return "controller/srv/InverseKinematics_Request";
}

template<>
struct has_fixed_size<controller::srv::InverseKinematics_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::InverseKinematics_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::InverseKinematics_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const InverseKinematics_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_angles
  {
    if (msg.joint_angles.size() == 0) {
      out << "joint_angles: []";
    } else {
      out << "joint_angles: [";
      size_t pending_items = msg.joint_angles.size();
      for (auto item : msg.joint_angles) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const InverseKinematics_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_angles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_angles.size() == 0) {
      out << "joint_angles: []\n";
    } else {
      out << "joint_angles:\n";
      for (auto item : msg.joint_angles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const InverseKinematics_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace controller

namespace rosidl_generator_traits
{

[[deprecated("use controller::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const controller::srv::InverseKinematics_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::InverseKinematics_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::InverseKinematics_Response>()
{
  return "controller::srv::InverseKinematics_Response";
}

template<>
inline const char * name<controller::srv::InverseKinematics_Response>()
{
  return "controller/srv/InverseKinematics_Response";
}

template<>
struct has_fixed_size<controller::srv::InverseKinematics_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::InverseKinematics_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::InverseKinematics_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::InverseKinematics>()
{
  return "controller::srv::InverseKinematics";
}

template<>
inline const char * name<controller::srv::InverseKinematics>()
{
  return "controller/srv/InverseKinematics";
}

template<>
struct has_fixed_size<controller::srv::InverseKinematics>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::InverseKinematics_Request>::value &&
    has_fixed_size<controller::srv::InverseKinematics_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::InverseKinematics>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::InverseKinematics_Request>::value &&
    has_bounded_size<controller::srv::InverseKinematics_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::InverseKinematics>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::InverseKinematics_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::InverseKinematics_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__TRAITS_HPP_
