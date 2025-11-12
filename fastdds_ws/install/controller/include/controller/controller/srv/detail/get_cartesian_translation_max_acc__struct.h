// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/GetCartesianTranslationMaxAcc.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_CARTESIAN_TRANSLATION_MAX_ACC__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__GET_CARTESIAN_TRANSLATION_MAX_ACC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetCartesianTranslationMaxAcc in the package controller.
typedef struct controller__srv__GetCartesianTranslationMaxAcc_Request
{
  uint8_t structure_needs_at_least_one_member;
} controller__srv__GetCartesianTranslationMaxAcc_Request;

// Struct for a sequence of controller__srv__GetCartesianTranslationMaxAcc_Request.
typedef struct controller__srv__GetCartesianTranslationMaxAcc_Request__Sequence
{
  controller__srv__GetCartesianTranslationMaxAcc_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__GetCartesianTranslationMaxAcc_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cartesian_params'
#include "controller/msg/detail/cartesian_params__struct.h"

/// Struct defined in srv/GetCartesianTranslationMaxAcc in the package controller.
typedef struct controller__srv__GetCartesianTranslationMaxAcc_Response
{
  controller__msg__CartesianParams__Sequence cartesian_params;
} controller__srv__GetCartesianTranslationMaxAcc_Response;

// Struct for a sequence of controller__srv__GetCartesianTranslationMaxAcc_Response.
typedef struct controller__srv__GetCartesianTranslationMaxAcc_Response__Sequence
{
  controller__srv__GetCartesianTranslationMaxAcc_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__GetCartesianTranslationMaxAcc_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__GET_CARTESIAN_TRANSLATION_MAX_ACC__STRUCT_H_
