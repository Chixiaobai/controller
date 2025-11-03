// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from controller:msg/TcpPayloadParams.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "controller/msg/detail/tcp_payload_params__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace controller
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TcpPayloadParams_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) controller::msg::TcpPayloadParams(_init);
}

void TcpPayloadParams_fini_function(void * message_memory)
{
  auto typed_message = static_cast<controller::msg::TcpPayloadParams *>(message_memory);
  typed_message->~TcpPayloadParams();
}

size_t size_function__TcpPayloadParams__parameters(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TcpPayloadParams__parameters(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__TcpPayloadParams__parameters(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__TcpPayloadParams__parameters(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TcpPayloadParams__parameters(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TcpPayloadParams__parameters(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TcpPayloadParams__parameters(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__TcpPayloadParams__parameters(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TcpPayloadParams_message_member_array[2] = {
  {
    "type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::TcpPayloadParams, type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "parameters",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::TcpPayloadParams, parameters),  // bytes offset in struct
    nullptr,  // default value
    size_function__TcpPayloadParams__parameters,  // size() function pointer
    get_const_function__TcpPayloadParams__parameters,  // get_const(index) function pointer
    get_function__TcpPayloadParams__parameters,  // get(index) function pointer
    fetch_function__TcpPayloadParams__parameters,  // fetch(index, &value) function pointer
    assign_function__TcpPayloadParams__parameters,  // assign(index, value) function pointer
    resize_function__TcpPayloadParams__parameters  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TcpPayloadParams_message_members = {
  "controller::msg",  // message namespace
  "TcpPayloadParams",  // message name
  2,  // number of fields
  sizeof(controller::msg::TcpPayloadParams),
  TcpPayloadParams_message_member_array,  // message members
  TcpPayloadParams_init_function,  // function to initialize message memory (memory has to be allocated)
  TcpPayloadParams_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TcpPayloadParams_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TcpPayloadParams_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace controller


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::msg::TcpPayloadParams>()
{
  return &::controller::msg::rosidl_typesupport_introspection_cpp::TcpPayloadParams_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, msg, TcpPayloadParams)() {
  return &::controller::msg::rosidl_typesupport_introspection_cpp::TcpPayloadParams_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
