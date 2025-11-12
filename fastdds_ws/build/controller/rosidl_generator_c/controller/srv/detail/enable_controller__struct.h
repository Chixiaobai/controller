// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/EnableController.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__ENABLE_CONTROLLER__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__ENABLE_CONTROLLER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/EnableController in the package controller.
typedef struct controller__srv__EnableController_Request
{
  bool enable;
} controller__srv__EnableController_Request;

// Struct for a sequence of controller__srv__EnableController_Request.
typedef struct controller__srv__EnableController_Request__Sequence
{
  controller__srv__EnableController_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__EnableController_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/EnableController in the package controller.
typedef struct controller__srv__EnableController_Response
{
  bool success;
} controller__srv__EnableController_Response;

// Struct for a sequence of controller__srv__EnableController_Response.
typedef struct controller__srv__EnableController_Response__Sequence
{
  controller__srv__EnableController_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__EnableController_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__ENABLE_CONTROLLER__STRUCT_H_
