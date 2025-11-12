// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/Localization.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__LOCALIZATION__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__LOCALIZATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/Localization in the package controller.
typedef struct controller__msg__Localization
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Pose pose;
  double confidence;
  /// 非0有效
  int8_t valid_flag;
  int32_t error_msgs;
  int32_t trajectory_id;
} controller__msg__Localization;

// Struct for a sequence of controller__msg__Localization.
typedef struct controller__msg__Localization__Sequence
{
  controller__msg__Localization * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__Localization__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__LOCALIZATION__STRUCT_H_
