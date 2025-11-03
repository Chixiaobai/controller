// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/MultiJointMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/multi_joint_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'joint_angles'
#include "controller/msg/detail/joint_angle__traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const MultiJointMove_Request & msg,
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
  const MultiJointMove_Request & msg,
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
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MultiJointMove_Request & msg, bool use_flow_style = false)
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
  const controller::srv::MultiJointMove_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::MultiJointMove_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::MultiJointMove_Request>()
{
  return "controller::srv::MultiJointMove_Request";
}

template<>
inline const char * name<controller::srv::MultiJointMove_Request>()
{
  return "controller/srv/MultiJointMove_Request";
}

template<>
struct has_fixed_size<controller::srv::MultiJointMove_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::MultiJointMove_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::MultiJointMove_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const MultiJointMove_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
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
  const MultiJointMove_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
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

inline std::string to_yaml(const MultiJointMove_Response & msg, bool use_flow_style = false)
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
  const controller::srv::MultiJointMove_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::MultiJointMove_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::MultiJointMove_Response>()
{
  return "controller::srv::MultiJointMove_Response";
}

template<>
inline const char * name<controller::srv::MultiJointMove_Response>()
{
  return "controller/srv/MultiJointMove_Response";
}

template<>
struct has_fixed_size<controller::srv::MultiJointMove_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::MultiJointMove_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::MultiJointMove_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::MultiJointMove>()
{
  return "controller::srv::MultiJointMove";
}

template<>
inline const char * name<controller::srv::MultiJointMove>()
{
  return "controller/srv/MultiJointMove";
}

template<>
struct has_fixed_size<controller::srv::MultiJointMove>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::MultiJointMove_Request>::value &&
    has_fixed_size<controller::srv::MultiJointMove_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::MultiJointMove>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::MultiJointMove_Request>::value &&
    has_bounded_size<controller::srv::MultiJointMove_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::MultiJointMove>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::MultiJointMove_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::MultiJointMove_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__TRAITS_HPP_
