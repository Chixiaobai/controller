// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "controller/msg/detail/tcp_pose_params__struct.h"
// Member 'reference_joint_angles'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/InverseKinematics in the package controller.
typedef struct controller__srv__InverseKinematics_Request
{
  controller__msg__TcpPoseParams__Sequence pose;
  rosidl_runtime_c__double__Sequence reference_joint_angles;
  bool if_use_whole_body;
} controller__srv__InverseKinematics_Request;

// Struct for a sequence of controller__srv__InverseKinematics_Request.
typedef struct controller__srv__InverseKinematics_Request__Sequence
{
  controller__srv__InverseKinematics_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__InverseKinematics_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'joint_angles'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/InverseKinematics in the package controller.
typedef struct controller__srv__InverseKinematics_Response
{
  rosidl_runtime_c__double__Sequence joint_angles;
} controller__srv__InverseKinematics_Response;

// Struct for a sequence of controller__srv__InverseKinematics_Response.
typedef struct controller__srv__InverseKinematics_Response__Sequence
{
  controller__srv__InverseKinematics_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__InverseKinematics_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__INVERSE_KINEMATICS__STRUCT_H_
