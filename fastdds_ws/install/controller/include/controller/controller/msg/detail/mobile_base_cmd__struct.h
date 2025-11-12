// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/MobileBaseCmd.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MobileBaseCmd in the package controller.
typedef struct controller__msg__MobileBaseCmd
{
  float linear_velocity;
  float angular_velocity;
  float lifetime;
} controller__msg__MobileBaseCmd;

// Struct for a sequence of controller__msg__MobileBaseCmd.
typedef struct controller__msg__MobileBaseCmd__Sequence
{
  controller__msg__MobileBaseCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__MobileBaseCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__MOBILE_BASE_CMD__STRUCT_H_
