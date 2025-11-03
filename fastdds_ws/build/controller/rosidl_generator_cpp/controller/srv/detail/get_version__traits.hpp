// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from controller:srv/GetVersion.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_VERSION__TRAITS_HPP_
#define CONTROLLER__SRV__DETAIL__GET_VERSION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "controller/srv/detail/get_version__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetVersion_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetVersion_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetVersion_Request & msg, bool use_flow_style = false)
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
  const controller::srv::GetVersion_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::GetVersion_Request & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::GetVersion_Request>()
{
  return "controller::srv::GetVersion_Request";
}

template<>
inline const char * name<controller::srv::GetVersion_Request>()
{
  return "controller/srv/GetVersion_Request";
}

template<>
struct has_fixed_size<controller::srv::GetVersion_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<controller::srv::GetVersion_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<controller::srv::GetVersion_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace controller
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetVersion_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: main
  {
    out << "main: ";
    rosidl_generator_traits::value_to_yaml(msg.main, out);
    out << ", ";
  }

  // member: plugin_names
  {
    if (msg.plugin_names.size() == 0) {
      out << "plugin_names: []";
    } else {
      out << "plugin_names: [";
      size_t pending_items = msg.plugin_names.size();
      for (auto item : msg.plugin_names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: plugin_versions
  {
    if (msg.plugin_versions.size() == 0) {
      out << "plugin_versions: []";
    } else {
      out << "plugin_versions: [";
      size_t pending_items = msg.plugin_versions.size();
      for (auto item : msg.plugin_versions) {
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
  const GetVersion_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: main
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "main: ";
    rosidl_generator_traits::value_to_yaml(msg.main, out);
    out << "\n";
  }

  // member: plugin_names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.plugin_names.size() == 0) {
      out << "plugin_names: []\n";
    } else {
      out << "plugin_names:\n";
      for (auto item : msg.plugin_names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: plugin_versions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.plugin_versions.size() == 0) {
      out << "plugin_versions: []\n";
    } else {
      out << "plugin_versions:\n";
      for (auto item : msg.plugin_versions) {
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

inline std::string to_yaml(const GetVersion_Response & msg, bool use_flow_style = false)
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
  const controller::srv::GetVersion_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  controller::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use controller::srv::to_yaml() instead")]]
inline std::string to_yaml(const controller::srv::GetVersion_Response & msg)
{
  return controller::srv::to_yaml(msg);
}

template<>
inline const char * data_type<controller::srv::GetVersion_Response>()
{
  return "controller::srv::GetVersion_Response";
}

template<>
inline const char * name<controller::srv::GetVersion_Response>()
{
  return "controller/srv/GetVersion_Response";
}

template<>
struct has_fixed_size<controller::srv::GetVersion_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<controller::srv::GetVersion_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<controller::srv::GetVersion_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<controller::srv::GetVersion>()
{
  return "controller::srv::GetVersion";
}

template<>
inline const char * name<controller::srv::GetVersion>()
{
  return "controller/srv/GetVersion";
}

template<>
struct has_fixed_size<controller::srv::GetVersion>
  : std::integral_constant<
    bool,
    has_fixed_size<controller::srv::GetVersion_Request>::value &&
    has_fixed_size<controller::srv::GetVersion_Response>::value
  >
{
};

template<>
struct has_bounded_size<controller::srv::GetVersion>
  : std::integral_constant<
    bool,
    has_bounded_size<controller::srv::GetVersion_Request>::value &&
    has_bounded_size<controller::srv::GetVersion_Response>::value
  >
{
};

template<>
struct is_service<controller::srv::GetVersion>
  : std::true_type
{
};

template<>
struct is_service_request<controller::srv::GetVersion_Request>
  : std::true_type
{
};

template<>
struct is_service_response<controller::srv::GetVersion_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CONTROLLER__SRV__DETAIL__GET_VERSION__TRAITS_HPP_
