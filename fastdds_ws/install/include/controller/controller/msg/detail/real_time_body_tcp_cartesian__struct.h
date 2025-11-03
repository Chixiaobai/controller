// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/RealTimeBodyTcpCartesian.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RealTimeBodyTcpCartesian in the package controller.
typedef struct controller__msg__RealTimeBodyTcpCartesian
{
  int32_t id;
  double left_arm[6];
  int8_t left_arm_valid;
  double right_arm[6];
  int8_t right_arm_valid;
  double torso[6];
  int8_t torso_valid;
  double time;
} controller__msg__RealTimeBodyTcpCartesian;

// Struct for a sequence of controller__msg__RealTimeBodyTcpCartesian.
typedef struct controller__msg__RealTimeBodyTcpCartesian__Sequence
{
  controller__msg__RealTimeBodyTcpCartesian * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__RealTimeBodyTcpCartesian__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__REAL_TIME_BODY_TCP_CARTESIAN__STRUCT_H_
