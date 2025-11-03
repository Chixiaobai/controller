// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from controller:srv/GetJointMechanicalLimit.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "controller/srv/detail/get_joint_mechanical_limit__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetJointMechanicalLimit_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetJointMechanicalLimit_Request_type_support_ids_t;

static const _GetJointMechanicalLimit_Request_type_support_ids_t _GetJointMechanicalLimit_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetJointMechanicalLimit_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetJointMechanicalLimit_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetJointMechanicalLimit_Request_type_support_symbol_names_t _GetJointMechanicalLimit_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, controller, srv, GetJointMechanicalLimit_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, GetJointMechanicalLimit_Request)),
  }
};

typedef struct _GetJointMechanicalLimit_Request_type_support_data_t
{
  void * data[2];
} _GetJointMechanicalLimit_Request_type_support_data_t;

static _GetJointMechanicalLimit_Request_type_support_data_t _GetJointMechanicalLimit_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetJointMechanicalLimit_Request_message_typesupport_map = {
  2,
  "controller",
  &_GetJointMechanicalLimit_Request_message_typesupport_ids.typesupport_identifier[0],
  &_GetJointMechanicalLimit_Request_message_typesupport_symbol_names.symbol_name[0],
  &_GetJointMechanicalLimit_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetJointMechanicalLimit_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetJointMechanicalLimit_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::srv::GetJointMechanicalLimit_Request>()
{
  return &::controller::srv::rosidl_typesupport_cpp::GetJointMechanicalLimit_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, controller, srv, GetJointMechanicalLimit_Request)() {
  return get_message_type_support_handle<controller::srv::GetJointMechanicalLimit_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "controller/srv/detail/get_joint_mechanical_limit__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetJointMechanicalLimit_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetJointMechanicalLimit_Response_type_support_ids_t;

static const _GetJointMechanicalLimit_Response_type_support_ids_t _GetJointMechanicalLimit_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetJointMechanicalLimit_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetJointMechanicalLimit_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetJointMechanicalLimit_Response_type_support_symbol_names_t _GetJointMechanicalLimit_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, controller, srv, GetJointMechanicalLimit_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, GetJointMechanicalLimit_Response)),
  }
};

typedef struct _GetJointMechanicalLimit_Response_type_support_data_t
{
  void * data[2];
} _GetJointMechanicalLimit_Response_type_support_data_t;

static _GetJointMechanicalLimit_Response_type_support_data_t _GetJointMechanicalLimit_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetJointMechanicalLimit_Response_message_typesupport_map = {
  2,
  "controller",
  &_GetJointMechanicalLimit_Response_message_typesupport_ids.typesupport_identifier[0],
  &_GetJointMechanicalLimit_Response_message_typesupport_symbol_names.symbol_name[0],
  &_GetJointMechanicalLimit_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t GetJointMechanicalLimit_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetJointMechanicalLimit_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::srv::GetJointMechanicalLimit_Response>()
{
  return &::controller::srv::rosidl_typesupport_cpp::GetJointMechanicalLimit_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, controller, srv, GetJointMechanicalLimit_Response)() {
  return get_message_type_support_handle<controller::srv::GetJointMechanicalLimit_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "controller/srv/detail/get_joint_mechanical_limit__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_cpp
{

typedef struct _GetJointMechanicalLimit_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _GetJointMechanicalLimit_type_support_ids_t;

static const _GetJointMechanicalLimit_type_support_ids_t _GetJointMechanicalLimit_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _GetJointMechanicalLimit_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _GetJointMechanicalLimit_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _GetJointMechanicalLimit_type_support_symbol_names_t _GetJointMechanicalLimit_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, controller, srv, GetJointMechanicalLimit)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, GetJointMechanicalLimit)),
  }
};

typedef struct _GetJointMechanicalLimit_type_support_data_t
{
  void * data[2];
} _GetJointMechanicalLimit_type_support_data_t;

static _GetJointMechanicalLimit_type_support_data_t _GetJointMechanicalLimit_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _GetJointMechanicalLimit_service_typesupport_map = {
  2,
  "controller",
  &_GetJointMechanicalLimit_service_typesupport_ids.typesupport_identifier[0],
  &_GetJointMechanicalLimit_service_typesupport_symbol_names.symbol_name[0],
  &_GetJointMechanicalLimit_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t GetJointMechanicalLimit_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_GetJointMechanicalLimit_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace srv

}  // namespace controller

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<controller::srv::GetJointMechanicalLimit>()
{
  return &::controller::srv::rosidl_typesupport_cpp::GetJointMechanicalLimit_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_cpp, controller, srv, GetJointMechanicalLimit)() {
  return ::rosidl_typesupport_cpp::get_service_type_support_handle<controller::srv::GetJointMechanicalLimit>();
}

#ifdef __cplusplus
}
#endif
