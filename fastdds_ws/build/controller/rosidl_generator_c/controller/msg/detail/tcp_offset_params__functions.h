// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from controller:msg/TcpOffsetParams.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__TCP_OFFSET_PARAMS__FUNCTIONS_H_
#define CONTROLLER__MSG__DETAIL__TCP_OFFSET_PARAMS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "controller/msg/rosidl_generator_c__visibility_control.h"

#include "controller/msg/detail/tcp_offset_params__struct.h"

/// Initialize msg/TcpOffsetParams message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * controller__msg__TcpOffsetParams
 * )) before or use
 * controller__msg__TcpOffsetParams__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__TcpOffsetParams__init(controller__msg__TcpOffsetParams * msg);

/// Finalize msg/TcpOffsetParams message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
void
controller__msg__TcpOffsetParams__fini(controller__msg__TcpOffsetParams * msg);

/// Create msg/TcpOffsetParams message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * controller__msg__TcpOffsetParams__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
controller__msg__TcpOffsetParams *
controller__msg__TcpOffsetParams__create();

/// Destroy msg/TcpOffsetParams message.
/**
 * It calls
 * controller__msg__TcpOffsetParams__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
void
controller__msg__TcpOffsetParams__destroy(controller__msg__TcpOffsetParams * msg);

/// Check for msg/TcpOffsetParams message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__TcpOffsetParams__are_equal(const controller__msg__TcpOffsetParams * lhs, const controller__msg__TcpOffsetParams * rhs);

/// Copy a msg/TcpOffsetParams message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__TcpOffsetParams__copy(
  const controller__msg__TcpOffsetParams * input,
  controller__msg__TcpOffsetParams * output);

/// Initialize array of msg/TcpOffsetParams messages.
/**
 * It allocates the memory for the number of elements and calls
 * controller__msg__TcpOffsetParams__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__TcpOffsetParams__Sequence__init(controller__msg__TcpOffsetParams__Sequence * array, size_t size);

/// Finalize array of msg/TcpOffsetParams messages.
/**
 * It calls
 * controller__msg__TcpOffsetParams__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
void
controller__msg__TcpOffsetParams__Sequence__fini(controller__msg__TcpOffsetParams__Sequence * array);

/// Create array of msg/TcpOffsetParams messages.
/**
 * It allocates the memory for the array and calls
 * controller__msg__TcpOffsetParams__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
controller__msg__TcpOffsetParams__Sequence *
controller__msg__TcpOffsetParams__Sequence__create(size_t size);

/// Destroy array of msg/TcpOffsetParams messages.
/**
 * It calls
 * controller__msg__TcpOffsetParams__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
void
controller__msg__TcpOffsetParams__Sequence__destroy(controller__msg__TcpOffsetParams__Sequence * array);

/// Check for msg/TcpOffsetParams message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__TcpOffsetParams__Sequence__are_equal(const controller__msg__TcpOffsetParams__Sequence * lhs, const controller__msg__TcpOffsetParams__Sequence * rhs);

/// Copy an array of msg/TcpOffsetParams messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__TcpOffsetParams__Sequence__copy(
  const controller__msg__TcpOffsetParams__Sequence * input,
  controller__msg__TcpOffsetParams__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__TCP_OFFSET_PARAMS__FUNCTIONS_H_
