// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from controller:srv/InverseKinematics.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "controller/srv/detail/inverse_kinematics__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void InverseKinematics_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) controller::srv::InverseKinematics_Request(_init);
}

void InverseKinematics_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<controller::srv::InverseKinematics_Request *>(message_memory);
  typed_message->~InverseKinematics_Request();
}

size_t size_function__InverseKinematics_Request__pose(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<controller::msg::TcpPoseParams> *>(untyped_member);
  return member->size();
}

const void * get_const_function__InverseKinematics_Request__pose(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<controller::msg::TcpPoseParams> *>(untyped_member);
  return &member[index];
}

void * get_function__InverseKinematics_Request__pose(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<controller::msg::TcpPoseParams> *>(untyped_member);
  return &member[index];
}

void fetch_function__InverseKinematics_Request__pose(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const controller::msg::TcpPoseParams *>(
    get_const_function__InverseKinematics_Request__pose(untyped_member, index));
  auto & value = *reinterpret_cast<controller::msg::TcpPoseParams *>(untyped_value);
  value = item;
}

void assign_function__InverseKinematics_Request__pose(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<controller::msg::TcpPoseParams *>(
    get_function__InverseKinematics_Request__pose(untyped_member, index));
  const auto & value = *reinterpret_cast<const controller::msg::TcpPoseParams *>(untyped_value);
  item = value;
}

void resize_function__InverseKinematics_Request__pose(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<controller::msg::TcpPoseParams> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember InverseKinematics_Request_message_member_array[2] = {
  {
    "pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<controller::msg::TcpPoseParams>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::srv::InverseKinematics_Request, pose),  // bytes offset in struct
    nullptr,  // default value
    size_function__InverseKinematics_Request__pose,  // size() function pointer
    get_const_function__InverseKinematics_Request__pose,  // get_const(index) function pointer
    get_function__InverseKinematics_Request__pose,  // get(index) function pointer
    fetch_function__InverseKinematics_Request__pose,  // fetch(index, &value) function pointer
    assign_function__InverseKinematics_Request__pose,  // assign(index, value) function pointer
    resize_function__InverseKinematics_Request__pose  // resize(index) function pointer
  },
  {
    "if_use_whole_body",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::srv::InverseKinematics_Request, if_use_whole_body),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers InverseKinematics_Request_message_members = {
  "controller::srv",  // message namespace
  "InverseKinematics_Request",  // message name
  2,  // number of fields
  sizeof(controller::srv::InverseKinematics_Request),
  InverseKinematics_Request_message_member_array,  // message members
  InverseKinematics_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  InverseKinematics_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t InverseKinematics_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &InverseKinematics_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace controller


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::srv::InverseKinematics_Request>()
{
  return &::controller::srv::rosidl_typesupport_introspection_cpp::InverseKinematics_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, InverseKinematics_Request)() {
  return &::controller::srv::rosidl_typesupport_introspection_cpp::InverseKinematics_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "controller/srv/detail/inverse_kinematics__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void InverseKinematics_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) controller::srv::InverseKinematics_Response(_init);
}

void InverseKinematics_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<controller::srv::InverseKinematics_Response *>(message_memory);
  typed_message->~InverseKinematics_Response();
}

size_t size_function__InverseKinematics_Response__joint_angles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<double> *>(untyped_member);
  return member->size();
}

const void * get_const_function__InverseKinematics_Response__joint_angles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<double> *>(untyped_member);
  return &member[index];
}

void * get_function__InverseKinematics_Response__joint_angles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<double> *>(untyped_member);
  return &member[index];
}

void fetch_function__InverseKinematics_Response__joint_angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__InverseKinematics_Response__joint_angles(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__InverseKinematics_Response__joint_angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__InverseKinematics_Response__joint_angles(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

void resize_function__InverseKinematics_Response__joint_angles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<double> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember InverseKinematics_Response_message_member_array[1] = {
  {
    "joint_angles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(controller::srv::InverseKinematics_Response, joint_angles),  // bytes offset in struct
    nullptr,  // default value
    size_function__InverseKinematics_Response__joint_angles,  // size() function pointer
    get_const_function__InverseKinematics_Response__joint_angles,  // get_const(index) function pointer
    get_function__InverseKinematics_Response__joint_angles,  // get(index) function pointer
    fetch_function__InverseKinematics_Response__joint_angles,  // fetch(index, &value) function pointer
    assign_function__InverseKinematics_Response__joint_angles,  // assign(index, value) function pointer
    resize_function__InverseKinematics_Response__joint_angles  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers InverseKinematics_Response_message_members = {
  "controller::srv",  // message namespace
  "InverseKinematics_Response",  // message name
  1,  // number of fields
  sizeof(controller::srv::InverseKinematics_Response),
  InverseKinematics_Response_message_member_array,  // message members
  InverseKinematics_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  InverseKinematics_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t InverseKinematics_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &InverseKinematics_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace controller


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<controller::srv::InverseKinematics_Response>()
{
  return &::controller::srv::rosidl_typesupport_introspection_cpp::InverseKinematics_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, InverseKinematics_Response)() {
  return &::controller::srv::rosidl_typesupport_introspection_cpp::InverseKinematics_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "controller/srv/detail/inverse_kinematics__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace controller
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers InverseKinematics_service_members = {
  "controller::srv",  // service namespace
  "InverseKinematics",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<controller::srv::InverseKinematics>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t InverseKinematics_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &InverseKinematics_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace controller


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<controller::srv::InverseKinematics>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::controller::srv::rosidl_typesupport_introspection_cpp::InverseKinematics_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::controller::srv::InverseKinematics_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::controller::srv::InverseKinematics_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, controller, srv, InverseKinematics)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<controller::srv::InverseKinematics>();
}

#ifdef __cplusplus
}
#endif
