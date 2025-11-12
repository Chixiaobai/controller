// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/SetJointMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_JOINT_MAX_VEL__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__SET_JOINT_MAX_VEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_params'
#include "controller/msg/detail/joint_params__struct.h"

/// Struct defined in srv/SetJointMaxVel in the package controller.
typedef struct controller__srv__SetJointMaxVel_Request
{
  controller__msg__JointParams__Sequence joint_params;
} controller__srv__SetJointMaxVel_Request;

// Struct for a sequence of controller__srv__SetJointMaxVel_Request.
typedef struct controller__srv__SetJointMaxVel_Request__Sequence
{
  controller__srv__SetJointMaxVel_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__SetJointMaxVel_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetJointMaxVel in the package controller.
typedef struct controller__srv__SetJointMaxVel_Response
{
  bool success;
} controller__srv__SetJointMaxVel_Response;

// Struct for a sequence of controller__srv__SetJointMaxVel_Response.
typedef struct controller__srv__SetJointMaxVel_Response__Sequence
{
  controller__srv__SetJointMaxVel_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__SetJointMaxVel_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__SET_JOINT_MAX_VEL__STRUCT_H_
