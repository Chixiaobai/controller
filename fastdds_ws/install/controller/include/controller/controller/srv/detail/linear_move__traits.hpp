// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/LinearMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__LINEAR_MOVE__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__LINEAR_MOVE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/linear_move__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'linear_move'
#include "controller/msg/detail/linear_move_params__traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const LinearMove_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: linear_move
  {
    if (msg.linear_move.size() == 0) {
      out << "linear_move: []";
    } else {
      out << "linear_move: [";
      size_t pending_items = msg.linear_move.size();
      for (auto item : msg.linear_move) {
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
  const LinearMove_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: linear_move
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.linear_move.size() == 0) {
      out << "linear_move: []\n";
    } else {
      out << "linear_move:\n";
      for (auto item : msg.linear_move) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LinearMove_Request & msg, bool use_flow_style = false)
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
  const controller::srv::LinearMove_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::LinearMove_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::LinearMove_Request>()
{
  return "controller::srv::LinearMove_Request";
}

template<>
inline const char * name<controller::srv::LinearMove_Request>()
{
  return "controller/srv/LinearMove_Request";
}

template<>
struct has_fixed_size<controller::srv::LinearMove_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::LinearMove_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::LinearMove_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const LinearMove_Response & msg,
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
  const LinearMove_Response & msg,
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

inline std::string to_yaml(const LinearMove_Response & msg, bool use_flow_style = false)
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
  const controller::srv::LinearMove_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::LinearMove_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::LinearMove_Response>()
{
  return "controller::srv::LinearMove_Response";
}

template<>
inline const char * name<controller::srv::LinearMove_Response>()
{
  return "controller/srv/LinearMove_Response";
}

template<>
struct has_fixed_size<controller::srv::LinearMove_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::LinearMove_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::LinearMove_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::LinearMove>()
{
  return "controller::srv::LinearMove";
}

template<>
inline const char * name<controller::srv::LinearMove>()
{
  return "controller/srv/LinearMove";
}

template<>
struct has_fixed_size<controller::srv::LinearMove>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::LinearMove_Request>::value &&
    has_fixed_size<controller::srv::LinearMove_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::LinearMove>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::LinearMove_Request>::value &&
    has_bounded_size<controller::srv::LinearMove_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::LinearMove>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::LinearMove_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::LinearMove_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__LINEAR_MOVE__TRAITS_HPP_
