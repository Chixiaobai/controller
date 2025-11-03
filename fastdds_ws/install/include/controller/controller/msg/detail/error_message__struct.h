// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:msg/ErrorMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__STRUCT_H_
#define CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'msg'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ErrorMessage in the package controller.
typedef struct controller__msg__ErrorMessage
{
  uint32_t id;
  uint8_t level;
  uint8_t module;
  int32_t error_code;
  rosidl_runtime_c__String msg;
} controller__msg__ErrorMessage;

// Struct for a sequence of controller__msg__ErrorMessage.
typedef struct controller__msg__ErrorMessage__Sequence
{
  controller__msg__ErrorMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__msg__ErrorMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__STRUCT_H_
