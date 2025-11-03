// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/OdometryMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/OdometryMessage in the package controller.
typedef struct controller__msg__OdometryMessage
{
  uint32_t accumulated_time;
  uint32_t accumulated_mileage;
} controller__msg__OdometryMessage;

// Struct for a sequence of controller__msg__OdometryMessage.
typedef struct controller__msg__OdometryMessage__Sequence
{
  controller__msg__OdometryMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__OdometryMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__ODOMETRY_MESSAGE__STRUCT_H_
