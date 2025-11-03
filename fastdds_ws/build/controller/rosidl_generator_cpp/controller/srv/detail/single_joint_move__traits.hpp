// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/SingleJointMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/single_joint_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const SingleJointMove_Request & msg,
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
  const SingleJointMove_Request & msg,
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

inline std::string to_yaml(const SingleJointMove_Request & msg, bool use_flow_style = false)
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
  const controller::srv::SingleJointMove_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::SingleJointMove_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::SingleJointMove_Request>()
{
  return "controller::srv::SingleJointMove_Request";
}

template<>
inline const char * name<controller::srv::SingleJointMove_Request>()
{
  return "controller/srv/SingleJointMove_Request";
}

template<>
struct has_fixed_size<controller::srv::SingleJointMove_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::SingleJointMove_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::SingleJointMove_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const SingleJointMove_Response & msg,
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
  const SingleJointMove_Response & msg,
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

inline std::string to_yaml(const SingleJointMove_Response & msg, bool use_flow_style = false)
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
  const controller::srv::SingleJointMove_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::SingleJointMove_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::SingleJointMove_Response>()
{
  return "controller::srv::SingleJointMove_Response";
}

template<>
inline const char * name<controller::srv::SingleJointMove_Response>()
{
  return "controller/srv/SingleJointMove_Response";
}

template<>
struct has_fixed_size<controller::srv::SingleJointMove_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::SingleJointMove_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::SingleJointMove_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::SingleJointMove>()
{
  return "controller::srv::SingleJointMove";
}

template<>
inline const char * name<controller::srv::SingleJointMove>()
{
  return "controller/srv/SingleJointMove";
}

template<>
struct has_fixed_size<controller::srv::SingleJointMove>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::SingleJointMove_Request>::value &&
    has_fixed_size<controller::srv::SingleJointMove_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::SingleJointMove>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::SingleJointMove_Request>::value &&
    has_bounded_size<controller::srv::SingleJointMove_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::SingleJointMove>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::SingleJointMove_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::SingleJointMove_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__TRAITS_HPP_
