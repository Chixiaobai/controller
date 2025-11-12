// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/MobileBaseState.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MobileBaseState in the package controller.
typedef struct controller__msg__MobileBaseState
{
  float linear_velocity;
  float angular_velocity;
} controller__msg__MobileBaseState;

// Struct for a sequence of controller__msg__MobileBaseState.
typedef struct controller__msg__MobileBaseState__Sequence
{
  controller__msg__MobileBaseState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__MobileBaseState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__MOBILE_BASE_STATE__STRUCT_H_
