// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/ForwardKinematics.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/forward_kinematics__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const ForwardKinematics_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: type
  {
    if (msg.type.size() == 0) {
      out << "type: []";
    } else {
      out << "type: [";
      size_t pending_items = msg.type.size();
      for (auto item : msg.type) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

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
  const ForwardKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.type.size() == 0) {
      out << "type: []\n";
    } else {
      out << "type:\n";
      for (auto item : msg.type) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

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

inline std::string to_yaml(const ForwardKinematics_Request & msg, bool use_flow_style = false)
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
  const controller::srv::ForwardKinematics_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::ForwardKinematics_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::ForwardKinematics_Request>()
{
  return "controller::srv::ForwardKinematics_Request";
}

template<>
inline const char * name<controller::srv::ForwardKinematics_Request>()
{
  return "controller/srv/ForwardKinematics_Request";
}

template<>
struct has_fixed_size<controller::srv::ForwardKinematics_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::ForwardKinematics_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::ForwardKinematics_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'pose'
#include "controller/msg/detail/tcp_pose_params__traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const ForwardKinematics_Response & msg,
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ForwardKinematics_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ForwardKinematics_Response & msg, bool use_flow_style = false)
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
  const controller::srv::ForwardKinematics_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::ForwardKinematics_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::ForwardKinematics_Response>()
{
  return "controller::srv::ForwardKinematics_Response";
}

template<>
inline const char * name<controller::srv::ForwardKinematics_Response>()
{
  return "controller/srv/ForwardKinematics_Response";
}

template<>
struct has_fixed_size<controller::srv::ForwardKinematics_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::ForwardKinematics_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::ForwardKinematics_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::ForwardKinematics>()
{
  return "controller::srv::ForwardKinematics";
}

template<>
inline const char * name<controller::srv::ForwardKinematics>()
{
  return "controller/srv/ForwardKinematics";
}

template<>
struct has_fixed_size<controller::srv::ForwardKinematics>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::ForwardKinematics_Request>::value &&
    has_fixed_size<controller::srv::ForwardKinematics_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::ForwardKinematics>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::ForwardKinematics_Request>::value &&
    has_bounded_size<controller::srv::ForwardKinematics_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::ForwardKinematics>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::ForwardKinematics_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::ForwardKinematics_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__TRAITS_HPP_
