// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/GetCartesianMechanicalRotationMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_CARTESIAN_MECHANICAL_ROTATION_MAX_VEL__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__GET_CARTESIAN_MECHANICAL_ROTATION_MAX_VEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetCartesianMechanicalRotationMaxVel in the package controller.
typedef struct controller__srv__GetCartesianMechanicalRotationMaxVel_Request
{
  uint8_t structure_needs_at_least_one_member;
} controller__srv__GetCartesianMechanicalRotationMaxVel_Request;

// Struct for a sequence of controller__srv__GetCartesianMechanicalRotationMaxVel_Request.
typedef struct controller__srv__GetCartesianMechanicalRotationMaxVel_Request__Sequence
{
  controller__srv__GetCartesianMechanicalRotationMaxVel_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__GetCartesianMechanicalRotationMaxVel_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cartesian_params'
#include "controller/msg/detail/cartesian_params__struct.h"

/// Struct defined in srv/GetCartesianMechanicalRotationMaxVel in the package controller.
typedef struct controller__srv__GetCartesianMechanicalRotationMaxVel_Response
{
  controller__msg__CartesianParams__Sequence cartesian_params;
} controller__srv__GetCartesianMechanicalRotationMaxVel_Response;

// Struct for a sequence of controller__srv__GetCartesianMechanicalRotationMaxVel_Response.
typedef struct controller__srv__GetCartesianMechanicalRotationMaxVel_Response__Sequence
{
  controller__srv__GetCartesianMechanicalRotationMaxVel_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__GetCartesianMechanicalRotationMaxVel_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__GET_CARTESIAN_MECHANICAL_ROTATION_MAX_VEL__STRUCT_H_
