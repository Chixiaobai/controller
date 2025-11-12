// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:msg/LinearMoveParams.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/msg/detail/linear_move_params__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/msg/detail/linear_move_params__functions.h"
#include "controller/msg/detail/linear_move_params__struct.h"


// Include directives for member types
// Member `pose`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__msg__LinearMoveParams__init(message_memory);
}

void controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_fini_function(void * message_memory)
{
  controller__msg__LinearMoveParams__fini(message_memory);
}

size_t controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__size_function__LinearMoveParams__pose(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__get_const_function__LinearMoveParams__pose(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__get_function__LinearMoveParams__pose(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__fetch_function__LinearMoveParams__pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__get_const_function__LinearMoveParams__pose(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__assign_function__LinearMoveParams__pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__get_function__LinearMoveParams__pose(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__resize_function__LinearMoveParams__pose(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_message_member_array[4] = {
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__LinearMoveParams, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__LinearMoveParams, pose),  // bytes offset in struct
    NULL,  // default value
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__size_function__LinearMoveParams__pose,  // size() function pointer
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__get_const_function__LinearMoveParams__pose,  // get_const(index) function pointer
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__get_function__LinearMoveParams__pose,  // get(index) function pointer
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__fetch_function__LinearMoveParams__pose,  // fetch(index, &value) function pointer
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__assign_function__LinearMoveParams__pose,  // assign(index, value) function pointer
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__resize_function__LinearMoveParams__pose  // resize(index) function pointer
  },
  {
    "velocity_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__LinearMoveParams, velocity_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "acceleration_percent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__LinearMoveParams, acceleration_percent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_message_members = {
  "controller__msg",  // message namespace
  "LinearMoveParams",  // message name
  4,  // number of fields
  sizeof(controller__msg__LinearMoveParams),
  controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_message_member_array,  // message members
  controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_message_type_support_handle = {
  0,
  &controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, msg, LinearMoveParams)() {
  if (!controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_message_type_support_handle.typesupport_identifier) {
    controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__msg__LinearMoveParams__rosidl_typesupport_introspection_c__LinearMoveParams_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
