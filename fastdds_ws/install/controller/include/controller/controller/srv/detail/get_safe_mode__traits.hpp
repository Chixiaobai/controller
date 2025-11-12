// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/GetSafeMode.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_SAFE_MODE__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__GET_SAFE_MODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/get_safe_mode__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSafeMode_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSafeMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSafeMode_Request & msg, bool use_flow_style = false)
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
  const controller::srv::GetSafeMode_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::GetSafeMode_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::GetSafeMode_Request>()
{
  return "controller::srv::GetSafeMode_Request";
}

template<>
inline const char * name<controller::srv::GetSafeMode_Request>()
{
  return "controller/srv/GetSafeMode_Request";
}

template<>
struct has_fixed_size<controller::srv::GetSafeMode_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::GetSafeMode_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::GetSafeMode_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSafeMode_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: enable
  {
    out << "enable: ";
    rosidl_generator_traits::value_to_yaml(msg.enable, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSafeMode_Response & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSafeMode_Response & msg, bool use_flow_style = false)
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
  const controller::srv::GetSafeMode_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::GetSafeMode_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::GetSafeMode_Response>()
{
  return "controller::srv::GetSafeMode_Response";
}

template<>
inline const char * name<controller::srv::GetSafeMode_Response>()
{
  return "controller/srv/GetSafeMode_Response";
}

template<>
struct has_fixed_size<controller::srv::GetSafeMode_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::GetSafeMode_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::GetSafeMode_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::GetSafeMode>()
{
  return "controller::srv::GetSafeMode";
}

template<>
inline const char * name<controller::srv::GetSafeMode>()
{
  return "controller/srv/GetSafeMode";
}

template<>
struct has_fixed_size<controller::srv::GetSafeMode>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::GetSafeMode_Request>::value &&
    has_fixed_size<controller::srv::GetSafeMode_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::GetSafeMode>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::GetSafeMode_Request>::value &&
    has_bounded_size<controller::srv::GetSafeMode_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::GetSafeMode>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::GetSafeMode_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::GetSafeMode_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__GET_SAFE_MODE__TRAITS_HPP_
