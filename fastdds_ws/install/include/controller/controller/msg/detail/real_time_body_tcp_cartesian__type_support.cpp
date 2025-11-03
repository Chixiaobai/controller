// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from controller:msg/RealTimeBodyTcpCartesian.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "controller/msg/detail/real_time_body_tcp_cartesian__struct.hpp"
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

void RealTimeBodyTcpCartesian_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) controller::msg::RealTimeBodyTcpCartesian(_init);
}

void RealTimeBodyTcpCartesian_fini_function(void * message_memory)
{
  auto typed_message = static_cast<controller::msg::RealTimeBodyTcpCartesian *>(message_memory);
  typed_message->~RealTimeBodyTcpCartesian();
}

size_t size_function__RealTimeBodyTcpCartesian__left_arm(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__RealTimeBodyTcpCartesian__left_arm(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__RealTimeBodyTcpCartesian__left_arm(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__RealTimeBodyTcpCartesian__left_arm(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RealTimeBodyTcpCartesian__left_arm(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RealTimeBodyTcpCartesian__left_arm(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RealTimeBodyTcpCartesian__left_arm(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__RealTimeBodyTcpCartesian__right_arm(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__RealTimeBodyTcpCartesian__right_arm(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__RealTimeBodyTcpCartesian__right_arm(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__RealTimeBodyTcpCartesian__right_arm(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RealTimeBodyTcpCartesian__right_arm(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RealTimeBodyTcpCartesian__right_arm(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RealTimeBodyTcpCartesian__right_arm(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__RealTimeBodyTcpCartesian__torso(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__RealTimeBodyTcpCartesian__torso(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__RealTimeBodyTcpCartesian__torso(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__RealTimeBodyTcpCartesian__torso(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RealTimeBodyTcpCartesian__torso(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RealTimeBodyTcpCartesian__torso(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RealTimeBodyTcpCartesian__torso(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RealTimeBodyTcpCartesian_message_member_array[8] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyTcpCartesian, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_arm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyTcpCartesian, left_arm),  // bytes offset in struct
    nullptr,  // default value
    size_function__RealTimeBodyTcpCartesian__left_arm,  // size() function pointer
    get_const_function__RealTimeBodyTcpCartesian__left_arm,  // get_const(index) function pointer
    get_function__RealTimeBodyTcpCartesian__left_arm,  // get(index) function pointer
    fetch_function__RealTimeBodyTcpCartesian__left_arm,  // fetch(index, &value) function pointer
    assign_function__RealTimeBodyTcpCartesian__left_arm,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "left_arm_valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyTcpCartesian, left_arm_valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_arm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyTcpCartesian, right_arm),  // bytes offset in struct
    nullptr,  // default value
    size_function__RealTimeBodyTcpCartesian__right_arm,  // size() function pointer
    get_const_function__RealTimeBodyTcpCartesian__right_arm,  // get_const(index) function pointer
    get_function__RealTimeBodyTcpCartesian__right_arm,  // get(index) function pointer
    fetch_function__RealTimeBodyTcpCartesian__right_arm,  // fetch(index, &value) function pointer
    assign_function__RealTimeBodyTcpCartesian__right_arm,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "right_arm_valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyTcpCartesian, right_arm_valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "torso",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyTcpCartesian, torso),  // bytes offset in struct
    nullptr,  // default value
    size_function__RealTimeBodyTcpCartesian__torso,  // size() function pointer
    get_const_function__RealTimeBodyTcpCartesian__torso,  // get_const(index) function pointer
    get_function__RealTimeBodyTcpCartesian__torso,  // get(index) function pointer
    fetch_function__RealTimeBodyTcpCartesian__torso,  // fetch(index, &value) function pointer
    assign_function__RealTimeBodyTcpCartesian__torso,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "torso_valid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyTcpCartesian, torso_valid),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyTcpCartesian, time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RealTimeBodyTcpCartesian_message_members = {
  "controller::msg",  // message namespace
  "RealTimeBodyTcpCartesian",  // message name
  8,  // number of fields
  sizeof(controller::msg::RealTimeBodyTcpCartesian),
  RealTimeBodyTcpCartesian_message_member_array,  // message members
  RealTimeBodyTcpCartesian_init_function,  // function to initialize message memory (memory has to be allocated)
  RealTimeBodyTcpCartesian_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RealTimeBodyTcpCartesian_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RealTimeBodyTcpCartesian_message_members,
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
get_message_type_support_handle<controller::msg::RealTimeBodyTcpCartesian>()
{
  return &::controller::msg::rosidl_typesupport_introspection_cpp::RealTimeBodyTcpCartesian_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, msg, RealTimeBodyTcpCartesian)() {
  return &::controller::msg::rosidl_typesupport_introspection_cpp::RealTimeBodyTcpCartesian_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
