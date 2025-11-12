// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from controller:srv/GetVersion.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "controller/srv/detail/get_version__rosidl_typesupport_introspection_c.h"
#include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "controller/srv/detail/get_version__functions.h"
#include "controller/srv/detail/get_version__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__srv__GetVersion_Request__init(message_memory);
}

void controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_fini_function(void * message_memory)
{
  controller__srv__GetVersion_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__GetVersion_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_message_members = {
  "controller__srv",  // message namespace
  "GetVersion_Request",  // message name
  1,  // number of fields
  sizeof(controller__srv__GetVersion_Request),
  controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_message_member_array,  // message members
  controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_message_type_support_handle = {
  0,
  &controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, GetVersion_Request)() {
  if (!controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_message_type_support_handle.typesupport_identifier) {
    controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__srv__GetVersion_Request__rosidl_typesupport_introspection_c__GetVersion_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "controller/srv/detail/get_version__rosidl_typesupport_introspection_c.h"
// already included above
// #include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "controller/srv/detail/get_version__functions.h"
// already included above
// #include "controller/srv/detail/get_version__struct.h"


// Include directives for member types
// Member `main`
// Member `plugin_names`
// Member `plugin_versions`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  controller__srv__GetVersion_Response__init(message_memory);
}

void controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_fini_function(void * message_memory)
{
  controller__srv__GetVersion_Response__fini(message_memory);
}

size_t controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__size_function__GetVersion_Response__plugin_names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_const_function__GetVersion_Response__plugin_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_function__GetVersion_Response__plugin_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__fetch_function__GetVersion_Response__plugin_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_const_function__GetVersion_Response__plugin_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__assign_function__GetVersion_Response__plugin_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_function__GetVersion_Response__plugin_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__resize_function__GetVersion_Response__plugin_names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__size_function__GetVersion_Response__plugin_versions(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_const_function__GetVersion_Response__plugin_versions(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_function__GetVersion_Response__plugin_versions(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__fetch_function__GetVersion_Response__plugin_versions(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_const_function__GetVersion_Response__plugin_versions(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__assign_function__GetVersion_Response__plugin_versions(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_function__GetVersion_Response__plugin_versions(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__resize_function__GetVersion_Response__plugin_versions(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_message_member_array[3] = {
  {
    "main",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__GetVersion_Response, main),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "plugin_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__GetVersion_Response, plugin_names),  // bytes offset in struct
    NULL,  // default value
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__size_function__GetVersion_Response__plugin_names,  // size() function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_const_function__GetVersion_Response__plugin_names,  // get_const(index) function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_function__GetVersion_Response__plugin_names,  // get(index) function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__fetch_function__GetVersion_Response__plugin_names,  // fetch(index, &value) function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__assign_function__GetVersion_Response__plugin_names,  // assign(index, value) function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__resize_function__GetVersion_Response__plugin_names  // resize(index) function pointer
  },
  {
    "plugin_versions",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller__srv__GetVersion_Response, plugin_versions),  // bytes offset in struct
    NULL,  // default value
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__size_function__GetVersion_Response__plugin_versions,  // size() function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_const_function__GetVersion_Response__plugin_versions,  // get_const(index) function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__get_function__GetVersion_Response__plugin_versions,  // get(index) function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__fetch_function__GetVersion_Response__plugin_versions,  // fetch(index, &value) function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__assign_function__GetVersion_Response__plugin_versions,  // assign(index, value) function pointer
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__resize_function__GetVersion_Response__plugin_versions  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_message_members = {
  "controller__srv",  // message namespace
  "GetVersion_Response",  // message name
  3,  // number of fields
  sizeof(controller__srv__GetVersion_Response),
  controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_message_member_array,  // message members
  controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_message_type_support_handle = {
  0,
  &controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, GetVersion_Response)() {
  if (!controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_message_type_support_handle.typesupport_identifier) {
    controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &controller__srv__GetVersion_Response__rosidl_typesupport_introspection_c__GetVersion_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "controller/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "controller/srv/detail/get_version__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_service_members = {
  "controller__srv",  // service namespace
  "GetVersion",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_Request_message_type_support_handle,
  NULL  // response message
  // controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_Response_message_type_support_handle
};

static rosidl_service_type_support_t controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_service_type_support_handle = {
  0,
  &controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, GetVersion_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, GetVersion_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_controller
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, GetVersion)() {
  if (!controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_service_type_support_handle.typesupport_identifier) {
    controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, GetVersion_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, GetVersion_Response)()->data;
  }

  return &controller__srv__detail__get_version__rosidl_typesupport_introspection_c__GetVersion_service_type_support_handle;
}
