// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/GetJointMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_JOINT_MAX_VEL__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__GET_JOINT_MAX_VEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/get_joint_max_vel__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetJointMaxVel_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetJointMaxVel_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetJointMaxVel_Request & msg, bool use_flow_style = false)
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
  const controller::srv::GetJointMaxVel_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::GetJointMaxVel_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::GetJointMaxVel_Request>()
{
  return "controller::srv::GetJointMaxVel_Request";
}

template<>
inline const char * name<controller::srv::GetJointMaxVel_Request>()
{
  return "controller/srv/GetJointMaxVel_Request";
}

template<>
struct has_fixed_size<controller::srv::GetJointMaxVel_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::GetJointMaxVel_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::GetJointMaxVel_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'joint_params'
#include "controller/msg/detail/joint_params__traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetJointMaxVel_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_params
  {
    if (msg.joint_params.size() == 0) {
      out << "joint_params: []";
    } else {
      out << "joint_params: [";
      size_t pending_items = msg.joint_params.size();
      for (auto item : msg.joint_params) {
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
  const GetJointMaxVel_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_params.size() == 0) {
      out << "joint_params: []\n";
    } else {
      out << "joint_params:\n";
      for (auto item : msg.joint_params) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetJointMaxVel_Response & msg, bool use_flow_style = false)
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
  const controller::srv::GetJointMaxVel_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::GetJointMaxVel_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::GetJointMaxVel_Response>()
{
  return "controller::srv::GetJointMaxVel_Response";
}

template<>
inline const char * name<controller::srv::GetJointMaxVel_Response>()
{
  return "controller/srv/GetJointMaxVel_Response";
}

template<>
struct has_fixed_size<controller::srv::GetJointMaxVel_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::GetJointMaxVel_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::GetJointMaxVel_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::GetJointMaxVel>()
{
  return "controller::srv::GetJointMaxVel";
}

template<>
inline const char * name<controller::srv::GetJointMaxVel>()
{
  return "controller/srv/GetJointMaxVel";
}

template<>
struct has_fixed_size<controller::srv::GetJointMaxVel>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::GetJointMaxVel_Request>::value &&
    has_fixed_size<controller::srv::GetJointMaxVel_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::GetJointMaxVel>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::GetJointMaxVel_Request>::value &&
    has_bounded_size<controller::srv::GetJointMaxVel_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::GetJointMaxVel>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::GetJointMaxVel_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::GetJointMaxVel_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__GET_JOINT_MAX_VEL__TRAITS_HPP_
