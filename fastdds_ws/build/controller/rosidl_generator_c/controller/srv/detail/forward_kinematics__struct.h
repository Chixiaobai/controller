// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/ForwardKinematics.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
// Member 'joint_angles'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/ForwardKinematics in the package controller.
typedef struct controller__srv__ForwardKinematics_Request
{
  rosidl_runtime_c__int32__Sequence type;
  rosidl_runtime_c__double__Sequence joint_angles;
} controller__srv__ForwardKinematics_Request;

// Struct for a sequence of controller__srv__ForwardKinematics_Request.
typedef struct controller__srv__ForwardKinematics_Request__Sequence
{
  controller__srv__ForwardKinematics_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__ForwardKinematics_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "controller/msg/detail/tcp_pose_params__struct.h"

/// Struct defined in srv/ForwardKinematics in the package controller.
typedef struct controller__srv__ForwardKinematics_Response
{
  controller__msg__TcpPoseParams__Sequence pose;
} controller__srv__ForwardKinematics_Response;

// Struct for a sequence of controller__srv__ForwardKinematics_Response.
typedef struct controller__srv__ForwardKinematics_Response__Sequence
{
  controller__srv__ForwardKinematics_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__ForwardKinematics_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__FORWARD_KINEMATICS__STRUCT_H_
