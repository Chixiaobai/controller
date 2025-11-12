// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/SetChassisMaxVel.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetChassisMaxVel in the package controller.
typedef struct controller__srv__SetChassisMaxVel_Request
{
  double linear_velocity;
  double angular_velocity;
} controller__srv__SetChassisMaxVel_Request;

// Struct for a sequence of controller__srv__SetChassisMaxVel_Request.
typedef struct controller__srv__SetChassisMaxVel_Request__Sequence
{
  controller__srv__SetChassisMaxVel_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__SetChassisMaxVel_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetChassisMaxVel in the package controller.
typedef struct controller__srv__SetChassisMaxVel_Response
{
  bool success;
} controller__srv__SetChassisMaxVel_Response;

// Struct for a sequence of controller__srv__SetChassisMaxVel_Response.
typedef struct controller__srv__SetChassisMaxVel_Response__Sequence
{
  controller__srv__SetChassisMaxVel_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__SetChassisMaxVel_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__SET_CHASSIS_MAX_VEL__STRUCT_H_
