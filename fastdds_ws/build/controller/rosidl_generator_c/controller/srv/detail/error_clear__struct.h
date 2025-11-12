// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/ErrorClear.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__ERROR_CLEAR__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__ERROR_CLEAR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ErrorClear in the package controller.
typedef struct controller__srv__ErrorClear_Request
{
  uint8_t structure_needs_at_least_one_member;
} controller__srv__ErrorClear_Request;

// Struct for a sequence of controller__srv__ErrorClear_Request.
typedef struct controller__srv__ErrorClear_Request__Sequence
{
  controller__srv__ErrorClear_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__ErrorClear_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ErrorClear in the package controller.
typedef struct controller__srv__ErrorClear_Response
{
  bool success;
} controller__srv__ErrorClear_Response;

// Struct for a sequence of controller__srv__ErrorClear_Response.
typedef struct controller__srv__ErrorClear_Response__Sequence
{
  controller__srv__ErrorClear_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__ErrorClear_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__ERROR_CLEAR__STRUCT_H_
