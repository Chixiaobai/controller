// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/TcpPoseParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__STRUCT_H_

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
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/TcpPoseParams in the package controller.
typedef struct controller__msg__TcpPoseParams
{
  int32_t type;
  rosidl_runtime_c__double__Sequence pose;
} controller__msg__TcpPoseParams;

// Struct for a sequence of controller__msg__TcpPoseParams.
typedef struct controller__msg__TcpPoseParams__Sequence
{
  controller__msg__TcpPoseParams * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__TcpPoseParams__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__TCP_POSE_PARAMS__STRUCT_H_
