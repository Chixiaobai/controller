// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/LinearMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__LINEAR_MOVE__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__LINEAR_MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'linear_move'
#include "controller/msg/detail/linear_move_params__struct.h"

/// Struct defined in srv/LinearMove in the package controller.
typedef struct controller__srv__LinearMove_Request
{
  controller__msg__LinearMoveParams__Sequence linear_move;
} controller__srv__LinearMove_Request;

// Struct for a sequence of controller__srv__LinearMove_Request.
typedef struct controller__srv__LinearMove_Request__Sequence
{
  controller__srv__LinearMove_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__LinearMove_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/LinearMove in the package controller.
typedef struct controller__srv__LinearMove_Response
{
  int32_t result;
  int32_t task_id;
} controller__srv__LinearMove_Response;

// Struct for a sequence of controller__srv__LinearMove_Response.
typedef struct controller__srv__LinearMove_Response__Sequence
{
  controller__srv__LinearMove_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__LinearMove_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__LINEAR_MOVE__STRUCT_H_
