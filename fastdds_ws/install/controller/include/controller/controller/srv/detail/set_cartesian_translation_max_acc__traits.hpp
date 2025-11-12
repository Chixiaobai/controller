// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/SetCartesianTranslationMaxAcc.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_ACC__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_ACC__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/set_cartesian_translation_max_acc__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'cartesian_params'
#include "controller/msg/detail/cartesian_params__traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetCartesianTranslationMaxAcc_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: cartesian_params
  {
    if (msg.cartesian_params.size() == 0) {
      out << "cartesian_params: []";
    } else {
      out << "cartesian_params: [";
      size_t pending_items = msg.cartesian_params.size();
      for (auto item : msg.cartesian_params) {
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
  const SetCartesianTranslationMaxAcc_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cartesian_params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cartesian_params.size() == 0) {
      out << "cartesian_params: []\n";
    } else {
      out << "cartesian_params:\n";
      for (auto item : msg.cartesian_params) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetCartesianTranslationMaxAcc_Request & msg, bool use_flow_style = false)
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
  const controller::srv::SetCartesianTranslationMaxAcc_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::SetCartesianTranslationMaxAcc_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::SetCartesianTranslationMaxAcc_Request>()
{
  return "controller::srv::SetCartesianTranslationMaxAcc_Request";
}

template<>
inline const char * name<controller::srv::SetCartesianTranslationMaxAcc_Request>()
{
  return "controller/srv/SetCartesianTranslationMaxAcc_Request";
}

template<>
struct has_fixed_size<controller::srv::SetCartesianTranslationMaxAcc_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::SetCartesianTranslationMaxAcc_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::SetCartesianTranslationMaxAcc_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetCartesianTranslationMaxAcc_Response & msg,
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
  const SetCartesianTranslationMaxAcc_Response & msg,
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

inline std::string to_yaml(const SetCartesianTranslationMaxAcc_Response & msg, bool use_flow_style = false)
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
  const controller::srv::SetCartesianTranslationMaxAcc_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::SetCartesianTranslationMaxAcc_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::SetCartesianTranslationMaxAcc_Response>()
{
  return "controller::srv::SetCartesianTranslationMaxAcc_Response";
}

template<>
inline const char * name<controller::srv::SetCartesianTranslationMaxAcc_Response>()
{
  return "controller/srv/SetCartesianTranslationMaxAcc_Response";
}

template<>
struct has_fixed_size<controller::srv::SetCartesianTranslationMaxAcc_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::SetCartesianTranslationMaxAcc_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::SetCartesianTranslationMaxAcc_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::SetCartesianTranslationMaxAcc>()
{
  return "controller::srv::SetCartesianTranslationMaxAcc";
}

template<>
inline const char * name<controller::srv::SetCartesianTranslationMaxAcc>()
{
  return "controller/srv/SetCartesianTranslationMaxAcc";
}

template<>
struct has_fixed_size<controller::srv::SetCartesianTranslationMaxAcc>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::SetCartesianTranslationMaxAcc_Request>::value &&
    has_fixed_size<controller::srv::SetCartesianTranslationMaxAcc_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::SetCartesianTranslationMaxAcc>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::SetCartesianTranslationMaxAcc_Request>::value &&
    has_bounded_size<controller::srv::SetCartesianTranslationMaxAcc_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::SetCartesianTranslationMaxAcc>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::SetCartesianTranslationMaxAcc_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::SetCartesianTranslationMaxAcc_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__SET_CARTESIAN_TRANSLATION_MAX_ACC__TRAITS_HPP_
