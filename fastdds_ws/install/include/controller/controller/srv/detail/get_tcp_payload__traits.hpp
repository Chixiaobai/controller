// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/GetTcpPayload.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_TCP_PAYLOAD__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__GET_TCP_PAYLOAD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/get_tcp_payload__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTcpPayload_Request & msg,
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetTcpPayload_Request & msg,
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTcpPayload_Request & msg, bool use_flow_style = false)
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
  const controller::srv::GetTcpPayload_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::GetTcpPayload_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::GetTcpPayload_Request>()
{
  return "controller::srv::GetTcpPayload_Request";
}

template<>
inline const char * name<controller::srv::GetTcpPayload_Request>()
{
  return "controller/srv/GetTcpPayload_Request";
}

template<>
struct has_fixed_size<controller::srv::GetTcpPayload_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::GetTcpPayload_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::GetTcpPayload_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'tcp_payload_params'
#include "controller/msg/detail/tcp_payload_params__traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetTcpPayload_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: tcp_payload_params
  {
    if (msg.tcp_payload_params.size() == 0) {
      out << "tcp_payload_params: []";
    } else {
      out << "tcp_payload_params: [";
      size_t pending_items = msg.tcp_payload_params.size();
      for (auto item : msg.tcp_payload_params) {
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
  const GetTcpPayload_Response & msg,
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

  // member: tcp_payload_params
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tcp_payload_params.size() == 0) {
      out << "tcp_payload_params: []\n";
    } else {
      out << "tcp_payload_params:\n";
      for (auto item : msg.tcp_payload_params) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetTcpPayload_Response & msg, bool use_flow_style = false)
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
  const controller::srv::GetTcpPayload_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::GetTcpPayload_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::GetTcpPayload_Response>()
{
  return "controller::srv::GetTcpPayload_Response";
}

template<>
inline const char * name<controller::srv::GetTcpPayload_Response>()
{
  return "controller/srv/GetTcpPayload_Response";
}

template<>
struct has_fixed_size<controller::srv::GetTcpPayload_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::GetTcpPayload_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::GetTcpPayload_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::GetTcpPayload>()
{
  return "controller::srv::GetTcpPayload";
}

template<>
inline const char * name<controller::srv::GetTcpPayload>()
{
  return "controller/srv/GetTcpPayload";
}

template<>
struct has_fixed_size<controller::srv::GetTcpPayload>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::GetTcpPayload_Request>::value &&
    has_fixed_size<controller::srv::GetTcpPayload_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::GetTcpPayload>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::GetTcpPayload_Request>::value &&
    has_bounded_size<controller::srv::GetTcpPayload_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::GetTcpPayload>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::GetTcpPayload_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::GetTcpPayload_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__GET_TCP_PAYLOAD__TRAITS_HPP_
