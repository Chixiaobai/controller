// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/GetVersion.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_VERSION__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__GET_VERSION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetVersion in the package controller.
typedef struct controller__srv__GetVersion_Request
{
  uint8_t structure_needs_at_least_one_member;
} controller__srv__GetVersion_Request;

// Struct for a sequence of controller__srv__GetVersion_Request.
typedef struct controller__srv__GetVersion_Request__Sequence
{
  controller__srv__GetVersion_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__GetVersion_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'main'
// Member 'plugin_names'
// Member 'plugin_versions'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/GetVersion in the package controller.
typedef struct controller__srv__GetVersion_Response
{
  rosidl_runtime_c__String main;
  rosidl_runtime_c__String__Sequence plugin_names;
  rosidl_runtime_c__String__Sequence plugin_versions;
} controller__srv__GetVersion_Response;

// Struct for a sequence of controller__srv__GetVersion_Response.
typedef struct controller__srv__GetVersion_Response__Sequence
{
  controller__srv__GetVersion_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__GetVersion_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__GET_VERSION__STRUCT_H_
