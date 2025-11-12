// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from controller:msg/RealTimeBodyJoints.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "controller/msg/detail/real_time_body_joints__struct.hpp"
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

void RealTimeBodyJoints_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) controller::msg::RealTimeBodyJoints(_init);
}

void RealTimeBodyJoints_fini_function(void * message_memory)
{
  auto typed_message = static_cast<controller::msg::RealTimeBodyJoints *>(message_memory);
  typed_message->~RealTimeBodyJoints();
}

size_t size_function__RealTimeBodyJoints__left_arm(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__RealTimeBodyJoints__left_arm(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__RealTimeBodyJoints__left_arm(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__RealTimeBodyJoints__left_arm(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RealTimeBodyJoints__left_arm(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RealTimeBodyJoints__left_arm(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RealTimeBodyJoints__left_arm(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__RealTimeBodyJoints__right_arm(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__RealTimeBodyJoints__right_arm(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__RealTimeBodyJoints__right_arm(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__RealTimeBodyJoints__right_arm(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RealTimeBodyJoints__right_arm(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RealTimeBodyJoints__right_arm(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RealTimeBodyJoints__right_arm(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__RealTimeBodyJoints__torso(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__RealTimeBodyJoints__torso(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__RealTimeBodyJoints__torso(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__RealTimeBodyJoints__torso(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__RealTimeBodyJoints__torso(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__RealTimeBodyJoints__torso(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__RealTimeBodyJoints__torso(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RealTimeBodyJoints_message_member_array[8] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyJoints, id),  // bytes offset in struct
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
    7,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyJoints, left_arm),  // bytes offset in struct
    nullptr,  // default value
    size_function__RealTimeBodyJoints__left_arm,  // size() function pointer
    get_const_function__RealTimeBodyJoints__left_arm,  // get_const(index) function pointer
    get_function__RealTimeBodyJoints__left_arm,  // get(index) function pointer
    fetch_function__RealTimeBodyJoints__left_arm,  // fetch(index, &value) function pointer
    assign_function__RealTimeBodyJoints__left_arm,  // assign(index, value) function pointer
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
    offsetof(controller::msg::RealTimeBodyJoints, left_arm_valid),  // bytes offset in struct
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
    7,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyJoints, right_arm),  // bytes offset in struct
    nullptr,  // default value
    size_function__RealTimeBodyJoints__right_arm,  // size() function pointer
    get_const_function__RealTimeBodyJoints__right_arm,  // get_const(index) function pointer
    get_function__RealTimeBodyJoints__right_arm,  // get(index) function pointer
    fetch_function__RealTimeBodyJoints__right_arm,  // fetch(index, &value) function pointer
    assign_function__RealTimeBodyJoints__right_arm,  // assign(index, value) function pointer
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
    offsetof(controller::msg::RealTimeBodyJoints, right_arm_valid),  // bytes offset in struct
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
    3,  // array size
    false,  // is upper bound
    offsetof(controller::msg::RealTimeBodyJoints, torso),  // bytes offset in struct
    nullptr,  // default value
    size_function__RealTimeBodyJoints__torso,  // size() function pointer
    get_const_function__RealTimeBodyJoints__torso,  // get_const(index) function pointer
    get_function__RealTimeBodyJoints__torso,  // get(index) function pointer
    fetch_function__RealTimeBodyJoints__torso,  // fetch(index, &value) function pointer
    assign_function__RealTimeBodyJoints__torso,  // assign(index, value) function pointer
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
    offsetof(controller::msg::RealTimeBodyJoints, torso_valid),  // bytes offset in struct
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
    offsetof(controller::msg::RealTimeBodyJoints, time),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RealTimeBodyJoints_message_members = {
  "controller::msg",  // message namespace
  "RealTimeBodyJoints",  // message name
  8,  // number of fields
  sizeof(controller::msg::RealTimeBodyJoints),
  RealTimeBodyJoints_message_member_array,  // message members
  RealTimeBodyJoints_init_function,  // function to initialize message memory (memory has to be allocated)
  RealTimeBodyJoints_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RealTimeBodyJoints_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RealTimeBodyJoints_message_members,
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
get_message_type_support_handle<controller::msg::RealTimeBodyJoints>()
{
  return &::controller::msg::rosidl_typesupport_introspection_cpp::RealTimeBodyJoints_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, msg, RealTimeBodyJoints)() {
  return &::controller::msg::rosidl_typesupport_introspection_cpp::RealTimeBodyJoints_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
