// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/SingleJointMove.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SingleJointMove in the package controller.
typedef struct controller__srv__SingleJointMove_Request
{
  uint32_t joint_index;
  double target_position;
  double velocity;
} controller__srv__SingleJointMove_Request;

// Struct for a sequence of controller__srv__SingleJointMove_Request.
typedef struct controller__srv__SingleJointMove_Request__Sequence
{
  controller__srv__SingleJointMove_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__SingleJointMove_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SingleJointMove in the package controller.
typedef struct controller__srv__SingleJointMove_Response
{
  bool success;
  uint32_t token;
} controller__srv__SingleJointMove_Response;

// Struct for a sequence of controller__srv__SingleJointMove_Response.
typedef struct controller__srv__SingleJointMove_Response__Sequence
{
  controller__srv__SingleJointMove_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__SingleJointMove_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__SINGLE_JOINT_MOVE__STRUCT_H_
