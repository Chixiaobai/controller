// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from controller:msg/MoveMessage.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "controller/msg/detail/move_message__struct.hpp"
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

void MoveMessage_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) controller::msg::MoveMessage(_init);
}

void MoveMessage_fini_function(void * message_memory)
{
  auto typed_message = static_cast<controller::msg::MoveMessage *>(message_memory);
  typed_message->~MoveMessage();
}

size_t size_function__MoveMessage__position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MoveMessage__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveMessage__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__MoveMessage__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MoveMessage__position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MoveMessage__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MoveMessage__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__MoveMessage__position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MoveMessage__velocity(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MoveMessage__velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveMessage__velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__MoveMessage__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__MoveMessage__velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__MoveMessage__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__MoveMessage__velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__MoveMessage__velocity(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MoveMessage__tcp_pose(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<controller::msg::TcpPoseParams> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MoveMessage__tcp_pose(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<controller::msg::TcpPoseParams> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveMessage__tcp_pose(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<controller::msg::TcpPoseParams> *>(untyped_member);
  return &member[index];
}

void fetch_function__MoveMessage__tcp_pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const controller::msg::TcpPoseParams *>(
    get_const_function__MoveMessage__tcp_pose(untyped_member, index));
  auto & value = *reinterpret_cast<controller::msg::TcpPoseParams *>(untyped_value);
  value = item;
}

void assign_function__MoveMessage__tcp_pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<controller::msg::TcpPoseParams *>(
    get_function__MoveMessage__tcp_pose(untyped_member, index));
  const auto & value = *reinterpret_cast<const controller::msg::TcpPoseParams *>(untyped_value);
  item = value;
}

void resize_function__MoveMessage__tcp_pose(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<controller::msg::TcpPoseParams> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveMessage_message_member_array[5] = {
  {
    "state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::MoveMessage, state),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::MoveMessage, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveMessage__position,  // size() function pointer
    get_const_function__MoveMessage__position,  // get_const(index) function pointer
    get_function__MoveMessage__position,  // get(index) function pointer
    fetch_function__MoveMessage__position,  // fetch(index, &value) function pointer
    assign_function__MoveMessage__position,  // assign(index, value) function pointer
    resize_function__MoveMessage__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::MoveMessage, velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveMessage__velocity,  // size() function pointer
    get_const_function__MoveMessage__velocity,  // get_const(index) function pointer
    get_function__MoveMessage__velocity,  // get(index) function pointer
    fetch_function__MoveMessage__velocity,  // fetch(index, &value) function pointer
    assign_function__MoveMessage__velocity,  // assign(index, value) function pointer
    resize_function__MoveMessage__velocity  // resize(index) function pointer
  },
  {
    "tcp_pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<controller::msg::TcpPoseParams>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::MoveMessage, tcp_pose),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveMessage__tcp_pose,  // size() function pointer
    get_const_function__MoveMessage__tcp_pose,  // get_const(index) function pointer
    get_function__MoveMessage__tcp_pose,  // get(index) function pointer
    fetch_function__MoveMessage__tcp_pose,  // fetch(index, &value) function pointer
    assign_function__MoveMessage__tcp_pose,  // assign(index, value) function pointer
    resize_function__MoveMessage__tcp_pose  // resize(index) function pointer
  },
  {
    "token",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::MoveMessage, token),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveMessage_message_members = {
  "controller::msg",  // message namespace
  "MoveMessage",  // message name
  5,  // number of fields
  sizeof(controller::msg::MoveMessage),
  MoveMessage_message_member_array,  // message members
  MoveMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveMessage_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveMessage_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveMessage_message_members,
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
get_message_type_support_handle<controller::msg::MoveMessage>()
{
  return &::controller::msg::rosidl_typesupport_introspection_cpp::MoveMessage_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, msg, MoveMessage)() {
  return &::controller::msg::rosidl_typesupport_introspection_cpp::MoveMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
