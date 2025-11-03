// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/MultiJointMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_angles'
#include "controller/msg/detail/joint_angle__struct.h"

/// Struct defined in srv/MultiJointMove in the package controller.
typedef struct controller__srv__MultiJointMove_Request
{
  controller__msg__JointAngle__Sequence joint_angles;
} controller__srv__MultiJointMove_Request;

// Struct for a sequence of controller__srv__MultiJointMove_Request.
typedef struct controller__srv__MultiJointMove_Request__Sequence
{
  controller__srv__MultiJointMove_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__MultiJointMove_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MultiJointMove in the package controller.
typedef struct controller__srv__MultiJointMove_Response
{
  bool success;
  uint32_t token;
} controller__srv__MultiJointMove_Response;

// Struct for a sequence of controller__srv__MultiJointMove_Response.
typedef struct controller__srv__MultiJointMove_Response__Sequence
{
  controller__srv__MultiJointMove_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__MultiJointMove_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__MULTI_JOINT_MOVE__STRUCT_H_
