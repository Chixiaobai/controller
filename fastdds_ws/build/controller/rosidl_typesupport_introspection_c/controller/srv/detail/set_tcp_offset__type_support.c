// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:srv/SetTcpOffset.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/srv/detail/set_tcp_offset__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/srv/detail/set_tcp_offset__functions.h"
#include "controller/srv/detail/set_tcp_offset__struct.h"


// Include directives for member types
// Member `tcp_offset_params`
#include "controller/msg/tcp_offset_params.h"
// Member `tcp_offset_params`
#include "controller/msg/detail/tcp_offset_params__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__srv__SetTcpOffset_Request__init(message_memory);
}

void controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_fini_function(void * message_memory)
{
  controller__srv__SetTcpOffset_Request__fini(message_memory);
}

size_t controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__size_function__SetTcpOffset_Request__tcp_offset_params(
  const void * untyped_member)
{
  const controller__msg__TcpOffsetParams__Sequence * member =
    (const controller__msg__TcpOffsetParams__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__get_const_function__SetTcpOffset_Request__tcp_offset_params(
  const void * untyped_member, size_t index)
{
  const controller__msg__TcpOffsetParams__Sequence * member =
    (const controller__msg__TcpOffsetParams__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__get_function__SetTcpOffset_Request__tcp_offset_params(
  void * untyped_member, size_t index)
{
  controller__msg__TcpOffsetParams__Sequence * member =
    (controller__msg__TcpOffsetParams__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__fetch_function__SetTcpOffset_Request__tcp_offset_params(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const controller__msg__TcpOffsetParams * item =
    ((const controller__msg__TcpOffsetParams *)
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__get_const_function__SetTcpOffset_Request__tcp_offset_params(untyped_member, index));
  controller__msg__TcpOffsetParams * value =
    (controller__msg__TcpOffsetParams *)(untyped_value);
  *value = *item;
}

void controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__assign_function__SetTcpOffset_Request__tcp_offset_params(
  void * untyped_member, size_t index, const void * untyped_value)
{
  controller__msg__TcpOffsetParams * item =
    ((controller__msg__TcpOffsetParams *)
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__get_function__SetTcpOffset_Request__tcp_offset_params(untyped_member, index));
  const controller__msg__TcpOffsetParams * value =
    (const controller__msg__TcpOffsetParams *)(untyped_value);
  *item = *value;
}

bool controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__resize_function__SetTcpOffset_Request__tcp_offset_params(
  void * untyped_member, size_t size)
{
  controller__msg__TcpOffsetParams__Sequence * member =
    (controller__msg__TcpOffsetParams__Sequence *)(untyped_member);
  controller__msg__TcpOffsetParams__Sequence__fini(member);
  return controller__msg__TcpOffsetParams__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_member_array[1] = {
  {
    "tcp_offset_params",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__SetTcpOffset_Request, tcp_offset_params),  // bytes offset in struct
    NULL,  // default value
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__size_function__SetTcpOffset_Request__tcp_offset_params,  // size() function pointer
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__get_const_function__SetTcpOffset_Request__tcp_offset_params,  // get_const(index) function pointer
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__get_function__SetTcpOffset_Request__tcp_offset_params,  // get(index) function pointer
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__fetch_function__SetTcpOffset_Request__tcp_offset_params,  // fetch(index, &value) function pointer
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__assign_function__SetTcpOffset_Request__tcp_offset_params,  // assign(index, value) function pointer
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__resize_function__SetTcpOffset_Request__tcp_offset_params  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_members = {
  "controller__srv",  // message namespace
  "SetTcpOffset_Request",  // message name
  1,  // number of fields
  sizeof(controller__srv__SetTcpOffset_Request),
  controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_member_array,  // message members
  controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_type_support_handle = {
  0,
  &controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, SetTcpOffset_Request)() {
  controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, msg, TcpOffsetParams)();
  if (!controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_type_support_handle.typesupport_identifier) {
    controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__srv__SetTcpOffset_Request__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "controller/srv/detail/set_tcp_offset__rosidl_typesupport_introspection_c.h"
// already included above
// #include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "controller/srv/detail/set_tcp_offset__functions.h"
// already included above
// #include "controller/srv/detail/set_tcp_offset__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__srv__SetTcpOffset_Response__init(message_memory);
}

void controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_fini_function(void * message_memory)
{
  controller__srv__SetTcpOffset_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__SetTcpOffset_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_members = {
  "controller__srv",  // message namespace
  "SetTcpOffset_Response",  // message name
  1,  // number of fields
  sizeof(controller__srv__SetTcpOffset_Response),
  controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_member_array,  // message members
  controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_type_support_handle = {
  0,
  &controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, SetTcpOffset_Response)() {
  if (!controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_type_support_handle.typesupport_identifier) {
    controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__srv__SetTcpOffset_Response__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "controller/srv/detail/set_tcp_offset__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_service_members = {
  "controller__srv",  // service namespace
  "SetTcpOffset",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_Request_message_type_support_handle,
  NULL  // response message
  // controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_Response_message_type_support_handle
};

static rosidl_service_type_support_t controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_service_type_support_handle = {
  0,
  &controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, SetTcpOffset_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, SetTcpOffset_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, SetTcpOffset)() {
  if (!controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_service_type_support_handle.typesupport_identifier) {
    controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, SetTcpOffset_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, SetTcpOffset_Response)()->data;
  }

  return &controller__srv__detail__set_tcp_offset__rosidl_typesupport_introspection_c__SetTcpOffset_service_type_support_handle;
}
