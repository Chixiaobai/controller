// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:msg/MobileBaseCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/msg/detail/mobile_base_cmd__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/msg/detail/mobile_base_cmd__functions.h"
#include "controller/msg/detail/mobile_base_cmd__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__msg__MobileBaseCmd__init(message_memory);
}

void controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_fini_function(void * message_memory)
{
  controller__msg__MobileBaseCmd__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_message_member_array[3] = {
  {
    "linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MobileBaseCmd, linear_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MobileBaseCmd, angular_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "lifetime",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__msg__MobileBaseCmd, lifetime),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_message_members = {
  "controller__msg",  // message namespace
  "MobileBaseCmd",  // message name
  3,  // number of fields
  sizeof(controller__msg__MobileBaseCmd),
  controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_message_member_array,  // message members
  controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_message_type_support_handle = {
  0,
  &controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, msg, MobileBaseCmd)() {
  if (!controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_message_type_support_handle.typesupport_identifier) {
    controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__msg__MobileBaseCmd__rosidl_typesupport_introspection_c__MobileBaseCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
