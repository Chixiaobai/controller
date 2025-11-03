// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/JointAngle.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/JointAngle in the package controller.
typedef struct controller__msg__JointAngle
{
  uint32_t joint_index;
  double target_position;
  double velocity;
} controller__msg__JointAngle;

// Struct for a sequence of controller__msg__JointAngle.
typedef struct controller__msg__JointAngle__Sequence
{
  controller__msg__JointAngle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__JointAngle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__JOINT_ANGLE__STRUCT_H_
