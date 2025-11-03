// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from controller:srv/GetTcpOffset.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__STRUCT_H_
#define CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'type'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/GetTcpOffset in the package controller.
typedef struct controller__srv__GetTcpOffset_Request
{
  rosidl_runtime_c__int32__Sequence type;
} controller__srv__GetTcpOffset_Request;

// Struct for a sequence of controller__srv__GetTcpOffset_Request.
typedef struct controller__srv__GetTcpOffset_Request__Sequence
{
  controller__srv__GetTcpOffset_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__GetTcpOffset_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'tcp_offset_params'
#include "controller/msg/detail/tcp_offset_params__struct.h"

/// Struct defined in srv/GetTcpOffset in the package controller.
typedef struct controller__srv__GetTcpOffset_Response
{
  bool success;
  controller__msg__TcpOffsetParams__Sequence tcp_offset_params;
} controller__srv__GetTcpOffset_Response;

// Struct for a sequence of controller__srv__GetTcpOffset_Response.
typedef struct controller__srv__GetTcpOffset_Response__Sequence
{
  controller__srv__GetTcpOffset_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} controller__srv__GetTcpOffset_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__SRV__DETAIL__GET_TCP_OFFSET__STRUCT_H_
