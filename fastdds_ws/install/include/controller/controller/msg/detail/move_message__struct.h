// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/MoveMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
// Member 'velocity'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'tcp_pose'
#include "controller/msg/detail/tcp_pose_params__struct.h"

/// Struct defined in msg/MoveMessage in the package controller.
typedef struct controller__msg__MoveMessage
{
  uint8_t state;
  rosidl_runtime_c__double__Sequence position;
  rosidl_runtime_c__double__Sequence velocity;
  controller__msg__TcpPoseParams__Sequence tcp_pose;
  uint32_t token;
} controller__msg__MoveMessage;

// Struct for a sequence of controller__msg__MoveMessage.
typedef struct controller__msg__MoveMessage__Sequence
{
  controller__msg__MoveMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__MoveMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__MOVE_MESSAGE__STRUCT_H_
