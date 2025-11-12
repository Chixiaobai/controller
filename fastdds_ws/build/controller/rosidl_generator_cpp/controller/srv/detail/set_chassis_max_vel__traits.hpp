// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/SetChassisMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/set_chassis_max_vel__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetChassisMaxVel_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: linear_velocity
  {
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << ", ";
  }

  // member: angular_velocity
  {
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetChassisMaxVel_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: linear_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "linear_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.linear_velocity, out);
    out << "\n";
  }

  // member: angular_velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angular_velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.angular_velocity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetChassisMaxVel_Request & msg, bool use_flow_style = false)
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
  const controller::srv::SetChassisMaxVel_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::SetChassisMaxVel_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::SetChassisMaxVel_Request>()
{
  return "controller::srv::SetChassisMaxVel_Request";
}

template<>
inline const char * name<controller::srv::SetChassisMaxVel_Request>()
{
  return "controller/srv/SetChassisMaxVel_Request";
}

template<>
struct has_fixed_size<controller::srv::SetChassisMaxVel_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::SetChassisMaxVel_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::SetChassisMaxVel_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetChassisMaxVel_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetChassisMaxVel_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetChassisMaxVel_Response & msg, bool use_flow_style = false)
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
  const controller::srv::SetChassisMaxVel_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::SetChassisMaxVel_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::SetChassisMaxVel_Response>()
{
  return "controller::srv::SetChassisMaxVel_Response";
}

template<>
inline const char * name<controller::srv::SetChassisMaxVel_Response>()
{
  return "controller/srv/SetChassisMaxVel_Response";
}

template<>
struct has_fixed_size<controller::srv::SetChassisMaxVel_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::SetChassisMaxVel_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::SetChassisMaxVel_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::SetChassisMaxVel>()
{
  return "controller::srv::SetChassisMaxVel";
}

template<>
inline const char * name<controller::srv::SetChassisMaxVel>()
{
  return "controller/srv/SetChassisMaxVel";
}

template<>
struct has_fixed_size<controller::srv::SetChassisMaxVel>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::SetChassisMaxVel_Request>::value &&
    has_fixed_size<controller::srv::SetChassisMaxVel_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::SetChassisMaxVel>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::SetChassisMaxVel_Request>::value &&
    has_bounded_size<controller::srv::SetChassisMaxVel_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::SetChassisMaxVel>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::SetChassisMaxVel_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::SetChassisMaxVel_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__TRAITS_HPP_
