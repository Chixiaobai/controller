// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/IsEnabledController.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__IS_ENABLED_CONTROLLER__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__IS_ENABLED_CONTROLLER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/is_enabled_controller__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsEnabledController_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsEnabledController_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const IsEnabledController_Request & msg, bool use_flow_style = false)
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
  const controller::srv::IsEnabledController_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::IsEnabledController_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::IsEnabledController_Request>()
{
  return "controller::srv::IsEnabledController_Request";
}

template<>
inline const char * name<controller::srv::IsEnabledController_Request>()
{
  return "controller/srv/IsEnabledController_Request";
}

template<>
struct has_fixed_size<controller::srv::IsEnabledController_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::IsEnabledController_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::IsEnabledController_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const IsEnabledController_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: enable
  {
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const IsEnabledController_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: enable
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
    out << "\n";
  }

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

inline std::string to_yaml(const IsEnabledController_Response & msg, bool use_flow_style = false)
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
  const controller::srv::IsEnabledController_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::IsEnabledController_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::IsEnabledController_Response>()
{
  return "controller::srv::IsEnabledController_Response";
}

template<>
inline const char * name<controller::srv::IsEnabledController_Response>()
{
  return "controller/srv/IsEnabledController_Response";
}

template<>
struct has_fixed_size<controller::srv::IsEnabledController_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::IsEnabledController_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::IsEnabledController_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::IsEnabledController>()
{
  return "controller::srv::IsEnabledController";
}

template<>
inline const char * name<controller::srv::IsEnabledController>()
{
  return "controller/srv/IsEnabledController";
}

template<>
struct has_fixed_size<controller::srv::IsEnabledController>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::IsEnabledController_Request>::value &&
    has_fixed_size<controller::srv::IsEnabledController_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::IsEnabledController>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::IsEnabledController_Request>::value &&
    has_bounded_size<controller::srv::IsEnabledController_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::IsEnabledController>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::IsEnabledController_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::IsEnabledController_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__IS_ENABLED_CONTROLLER__TRAITS_HPP_
