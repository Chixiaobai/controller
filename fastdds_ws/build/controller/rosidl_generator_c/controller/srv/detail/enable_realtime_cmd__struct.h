// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/EnableRealtimeCmd.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__ENABLE_REALTIME_CMD__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__ENABLE_REALTIME_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/EnableRealtimeCmd in the package controller.
typedef struct controller__srv__EnableRealtimeCmd_Request
{
  bool enable;
} controller__srv__EnableRealtimeCmd_Request;

// Struct for a sequence of controller__srv__EnableRealtimeCmd_Request.
typedef struct controller__srv__EnableRealtimeCmd_Request__Sequence
{
  controller__srv__EnableRealtimeCmd_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__EnableRealtimeCmd_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/EnableRealtimeCmd in the package controller.
typedef struct controller__srv__EnableRealtimeCmd_Response
{
  bool success;
} controller__srv__EnableRealtimeCmd_Response;

// Struct for a sequence of controller__srv__EnableRealtimeCmd_Response.
typedef struct controller__srv__EnableRealtimeCmd_Response__Sequence
{
  controller__srv__EnableRealtimeCmd_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__EnableRealtimeCmd_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__ENABLE_REALTIME_CMD__STRUCT_H_
