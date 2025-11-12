// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:msg/MoveMessage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/msg/detail/move_message__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/msg/detail/move_message__functions.h"
#include "controller/msg/detail/move_message__struct.h"


// Include directives for member types
// Member `position`
// Member `velocity`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `tcp_pose`
#include "controller/msg/tcp_pose_params.h"
// Member `tcp_pose`
#include "controller/msg/detail/tcp_pose_params__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__msg__MoveMessage__init(message_memory);
}

void controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_fini_function(void * message_memory)
{
  controller__msg__MoveMessage__fini(message_memory);
}

size_t controller__msg__MoveMessage__rosidl_typesupport_introspection_c__size_function__MoveMessage__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__MoveMessage__rosidl_typesupport_introspection_c__fetch_function__MoveMessage__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void controller__msg__MoveMessage__rosidl_typesupport_introspection_c__assign_function__MoveMessage__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool controller__msg__MoveMessage__rosidl_typesupport_introspection_c__resize_function__MoveMessage__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t controller__msg__MoveMessage__rosidl_typesupport_introspection_c__size_function__MoveMessage__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__MoveMessage__rosidl_typesupport_introspection_c__fetch_function__MoveMessage__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void controller__msg__MoveMessage__rosidl_typesupport_introspection_c__assign_function__MoveMessage__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool controller__msg__MoveMessage__rosidl_typesupport_introspection_c__resize_function__MoveMessage__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t controller__msg__MoveMessage__rosidl_typesupport_introspection_c__size_function__MoveMessage__tcp_pose(
  const void * untyped_member)
{
  const controller__msg__TcpPoseParams__Sequence * member =
    (const controller__msg__TcpPoseParams__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__tcp_pose(
  const void * untyped_member, size_t index)
{
  const controller__msg__TcpPoseParams__Sequence * member =
    (const controller__msg__TcpPoseParams__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__tcp_pose(
  void * untyped_member, size_t index)
{
  controller__msg__TcpPoseParams__Sequence * member =
    (controller__msg__TcpPoseParams__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__MoveMessage__rosidl_typesupport_introspection_c__fetch_function__MoveMessage__tcp_pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const controller__msg__TcpPoseParams * item =
    ((const controller__msg__TcpPoseParams *)
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__tcp_pose(untyped_member, index));
  controller__msg__TcpPoseParams * value =
    (controller__msg__TcpPoseParams *)(untyped_value);
  *value = *item;
}

void controller__msg__MoveMessage__rosidl_typesupport_introspection_c__assign_function__MoveMessage__tcp_pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  controller__msg__TcpPoseParams * item =
    ((controller__msg__TcpPoseParams *)
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__tcp_pose(untyped_member, index));
  const controller__msg__TcpPoseParams * value =
    (const controller__msg__TcpPoseParams *)(untyped_value);
  *item = *value;
}

bool controller__msg__MoveMessage__rosidl_typesupport_introspection_c__resize_function__MoveMessage__tcp_pose(
  void * untyped_member, size_t size)
{
  controller__msg__TcpPoseParams__Sequence * member =
    (controller__msg__TcpPoseParams__Sequence *)(untyped_member);
  controller__msg__TcpPoseParams__Sequence__fini(member);
  return controller__msg__TcpPoseParams__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_member_array[5] = {
  {
    "state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MoveMessage, state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MoveMessage, position),  // bytes offset in struct
    NULL,  // default value
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__size_function__MoveMessage__position,  // size() function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__position,  // get_const(index) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__position,  // get(index) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__fetch_function__MoveMessage__position,  // fetch(index, &value) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__assign_function__MoveMessage__position,  // assign(index, value) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__resize_function__MoveMessage__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MoveMessage, velocity),  // bytes offset in struct
    NULL,  // default value
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__size_function__MoveMessage__velocity,  // size() function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__velocity,  // get_const(index) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__velocity,  // get(index) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__fetch_function__MoveMessage__velocity,  // fetch(index, &value) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__assign_function__MoveMessage__velocity,  // assign(index, value) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__resize_function__MoveMessage__velocity  // resize(index) function pointer
  },
  {
    "tcp_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MoveMessage, tcp_pose),  // bytes offset in struct
    NULL,  // default value
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__size_function__MoveMessage__tcp_pose,  // size() function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_const_function__MoveMessage__tcp_pose,  // get_const(index) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__get_function__MoveMessage__tcp_pose,  // get(index) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__fetch_function__MoveMessage__tcp_pose,  // fetch(index, &value) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__assign_function__MoveMessage__tcp_pose,  // assign(index, value) function pointer
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__resize_function__MoveMessage__tcp_pose  // resize(index) function pointer
  },
  {
    "token",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MoveMessage, token),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_members = {
  "controller__msg",  // message namespace
  "MoveMessage",  // message name
  5,  // number of fields
  sizeof(controller__msg__MoveMessage),
  controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_member_array,  // message members
  controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_type_support_handle = {
  0,
  &controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, msg, MoveMessage)() {
  controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, msg, TcpPoseParams)();
  if (!controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_type_support_handle.typesupport_identifier) {
    controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__msg__MoveMessage__rosidl_typesupport_introspection_c__MoveMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
