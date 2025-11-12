// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from controller:msg/RealTimeBodyTcpCartesian.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/real_time_body_tcp_cartesian__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "controller/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "controller/msg/detail/real_time_body_tcp_cartesian__struct.h"
#include "controller/msg/detail/real_time_body_tcp_cartesian__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _RealTimeBodyTcpCartesian__ros_msg_type = controller__msg__RealTimeBodyTcpCartesian;

static bool _RealTimeBodyTcpCartesian__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RealTimeBodyTcpCartesian__ros_msg_type * ros_message = static_cast<const _RealTimeBodyTcpCartesian__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: left_arm
  {
    size_t size = 6;
    auto array_ptr = ros_message->left_arm;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: left_arm_valid
  {
    cdr << ros_message->left_arm_valid;
  }

  // Field name: right_arm
  {
    size_t size = 6;
    auto array_ptr = ros_message->right_arm;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: right_arm_valid
  {
    cdr << ros_message->right_arm_valid;
  }

  // Field name: torso
  {
    size_t size = 6;
    auto array_ptr = ros_message->torso;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: torso_valid
  {
    cdr << ros_message->torso_valid;
  }

  // Field name: time
  {
    cdr << ros_message->time;
  }

  return true;
}

static bool _RealTimeBodyTcpCartesian__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RealTimeBodyTcpCartesian__ros_msg_type * ros_message = static_cast<_RealTimeBodyTcpCartesian__ros_msg_type *>(untyped_ros_message);
  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: left_arm
  {
    size_t size = 6;
    auto array_ptr = ros_message->left_arm;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: left_arm_valid
  {
    cdr >> ros_message->left_arm_valid;
  }

  // Field name: right_arm
  {
    size_t size = 6;
    auto array_ptr = ros_message->right_arm;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: right_arm_valid
  {
    cdr >> ros_message->right_arm_valid;
  }

  // Field name: torso
  {
    size_t size = 6;
    auto array_ptr = ros_message->torso;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: torso_valid
  {
    cdr >> ros_message->torso_valid;
  }

  // Field name: time
  {
    cdr >> ros_message->time;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t get_serialized_size_controller__msg__RealTimeBodyTcpCartesian(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RealTimeBodyTcpCartesian__ros_msg_type * ros_message = static_cast<const _RealTimeBodyTcpCartesian__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_arm
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->left_arm;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name left_arm_valid
  {
    size_t item_size = sizeof(ros_message->left_arm_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_arm
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->right_arm;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name right_arm_valid
  {
    size_t item_size = sizeof(ros_message->right_arm_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name torso
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->torso;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name torso_valid
  {
    size_t item_size = sizeof(ros_message->torso_valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name time
  {
    size_t item_size = sizeof(ros_message->time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _RealTimeBodyTcpCartesian__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_controller__msg__RealTimeBodyTcpCartesian(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_controller
size_t max_serialized_size_controller__msg__RealTimeBodyTcpCartesian(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: left_arm
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: left_arm_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: right_arm
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: right_arm_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: torso
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: torso_valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = controller__msg__RealTimeBodyTcpCartesian;
    is_plain =
      (
      offsetof(DataType, time) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RealTimeBodyTcpCartesian__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_controller__msg__RealTimeBodyTcpCartesian(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RealTimeBodyTcpCartesian = {
  "controller::msg",
  "RealTimeBodyTcpCartesian",
  _RealTimeBodyTcpCartesian__cdr_serialize,
  _RealTimeBodyTcpCartesian__cdr_deserialize,
  _RealTimeBodyTcpCartesian__get_serialized_size,
  _RealTimeBodyTcpCartesian__max_serialized_size
};

static rosidl_message_type_support_t _RealTimeBodyTcpCartesian__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RealTimeBodyTcpCartesian,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, controller, msg, RealTimeBodyTcpCartesian)() {
  return &_RealTimeBodyTcpCartesian__type_support;
}

#if defined(__cplusplus)
}
#endif
