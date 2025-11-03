// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/CartesianParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CartesianParams in the package controller.
typedef struct controller__msg__CartesianParams
{
  uint32_t cartesian_index;
  double trans_max_vel;
  double trans_max_acc;
  double rota_max_vel;
  double rota_max_acc;
} controller__msg__CartesianParams;

// Struct for a sequence of controller__msg__CartesianParams.
typedef struct controller__msg__CartesianParams__Sequence
{
  controller__msg__CartesianParams * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__CartesianParams__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__CARTESIAN_PARAMS__STRUCT_H_
