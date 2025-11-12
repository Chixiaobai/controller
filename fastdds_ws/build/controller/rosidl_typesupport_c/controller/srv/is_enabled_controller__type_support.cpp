// generated from rosidl_typesupport_c/resource/idl__type_support.cpp.em
// with input from controller:srv/IsEnabledController.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "controller/srv/detail/is_enabled_controller__struct.h"
#include "controller/srv/detail/is_enabled_controller__type_support.h"
#include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/message_type_support_dispatch.h"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_c/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _IsEnabledController_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _IsEnabledController_Request_type_support_ids_t;

static const _IsEnabledController_Request_type_support_ids_t _IsEnabledController_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _IsEnabledController_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _IsEnabledController_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _IsEnabledController_Request_type_support_symbol_names_t _IsEnabledController_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, controller, srv, IsEnabledController_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, IsEnabledController_Request)),
  }
};

typedef struct _IsEnabledController_Request_type_support_data_t
{
  void * data[2];
} _IsEnabledController_Request_type_support_data_t;

static _IsEnabledController_Request_type_support_data_t _IsEnabledController_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _IsEnabledController_Request_message_typesupport_map = {
  2,
  "controller",
  &_IsEnabledController_Request_message_typesupport_ids.typesupport_identifier[0],
  &_IsEnabledController_Request_message_typesupport_symbol_names.symbol_name[0],
  &_IsEnabledController_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t IsEnabledController_Request_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_IsEnabledController_Request_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace controller

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, controller, srv, IsEnabledController_Request)() {
  return &::controller::srv::rosidl_typesupport_c::IsEnabledController_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "controller/srv/detail/is_enabled_controller__struct.h"
// already included above
// #include "controller/srv/detail/is_enabled_controller__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
// already included above
// #include "rosidl_typesupport_c/message_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_c/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _IsEnabledController_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _IsEnabledController_Response_type_support_ids_t;

static const _IsEnabledController_Response_type_support_ids_t _IsEnabledController_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _IsEnabledController_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _IsEnabledController_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _IsEnabledController_Response_type_support_symbol_names_t _IsEnabledController_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, controller, srv, IsEnabledController_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, IsEnabledController_Response)),
  }
};

typedef struct _IsEnabledController_Response_type_support_data_t
{
  void * data[2];
} _IsEnabledController_Response_type_support_data_t;

static _IsEnabledController_Response_type_support_data_t _IsEnabledController_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _IsEnabledController_Response_message_typesupport_map = {
  2,
  "controller",
  &_IsEnabledController_Response_message_typesupport_ids.typesupport_identifier[0],
  &_IsEnabledController_Response_message_typesupport_symbol_names.symbol_name[0],
  &_IsEnabledController_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t IsEnabledController_Response_message_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_IsEnabledController_Response_message_typesupport_map),
  rosidl_typesupport_c__get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace controller

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_c, controller, srv, IsEnabledController_Response)() {
  return &::controller::srv::rosidl_typesupport_c::IsEnabledController_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "controller/srv/detail/is_enabled_controller__type_support.h"
// already included above
// #include "rosidl_typesupport_c/identifier.h"
#include "rosidl_typesupport_c/service_type_support_dispatch.h"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_c
{

typedef struct _IsEnabledController_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _IsEnabledController_type_support_ids_t;

static const _IsEnabledController_type_support_ids_t _IsEnabledController_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_c",  // ::rosidl_typesupport_fastrtps_c::typesupport_identifier,
    "rosidl_typesupport_introspection_c",  // ::rosidl_typesupport_introspection_c::typesupport_identifier,
  }
};

typedef struct _IsEnabledController_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _IsEnabledController_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _IsEnabledController_type_support_symbol_names_t _IsEnabledController_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, controller, srv, IsEnabledController)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, controller, srv, IsEnabledController)),
  }
};

typedef struct _IsEnabledController_type_support_data_t
{
  void * data[2];
} _IsEnabledController_type_support_data_t;

static _IsEnabledController_type_support_data_t _IsEnabledController_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _IsEnabledController_service_typesupport_map = {
  2,
  "controller",
  &_IsEnabledController_service_typesupport_ids.typesupport_identifier[0],
  &_IsEnabledController_service_typesupport_symbol_names.symbol_name[0],
  &_IsEnabledController_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t IsEnabledController_service_type_support_handle = {
  rosidl_typesupport_c__typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_IsEnabledController_service_typesupport_map),
  rosidl_typesupport_c__get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_c

}  // namespace srv

}  // namespace controller

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_c, controller, srv, IsEnabledController)() {
  return &::controller::srv::rosidl_typesupport_c::IsEnabledController_service_type_support_handle;
}

#ifdef __cplusplus
}
#endif
