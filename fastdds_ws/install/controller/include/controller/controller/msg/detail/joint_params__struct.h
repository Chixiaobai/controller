// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/JointParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__JOINT_PARAMS__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__JOINT_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/JointParams in the package controller.
typedef struct controller__msg__JointParams
{
  uint32_t joint_index;
  double max_pos;
  double min_pos;
  double max_vel;
  double max_acc;
} controller__msg__JointParams;

// Struct for a sequence of controller__msg__JointParams.
typedef struct controller__msg__JointParams__Sequence
{
  controller__msg__JointParams * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__JointParams__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__JOINT_PARAMS__STRUCT_H_
