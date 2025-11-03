// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from controller:msg/ErrorMessage.idl
// generated code does not contain a copyright notice

#ifndef CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__FUNCTIONS_H_
#define CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "controller/msg/rosidl_generator_c__visibility_control.h"

#include "controller/msg/detail/error_message__struct.h"

/// Initialize msg/ErrorMessage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * controller__msg__ErrorMessage
 * )) before or use
 * controller__msg__ErrorMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__ErrorMessage__init(controller__msg__ErrorMessage * msg);

/// Finalize msg/ErrorMessage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
void
controller__msg__ErrorMessage__fini(controller__msg__ErrorMessage * msg);

/// Create msg/ErrorMessage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * controller__msg__ErrorMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
controller__msg__ErrorMessage *
controller__msg__ErrorMessage__create();

/// Destroy msg/ErrorMessage message.
/**
 * It calls
 * controller__msg__ErrorMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
void
controller__msg__ErrorMessage__destroy(controller__msg__ErrorMessage * msg);

/// Check for msg/ErrorMessage message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__ErrorMessage__are_equal(const controller__msg__ErrorMessage * lhs, const controller__msg__ErrorMessage * rhs);

/// Copy a msg/ErrorMessage message.
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
controller__msg__ErrorMessage__copy(
  const controller__msg__ErrorMessage * input,
  controller__msg__ErrorMessage * output);

/// Initialize array of msg/ErrorMessage messages.
/**
 * It allocates the memory for the number of elements and calls
 * controller__msg__ErrorMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__ErrorMessage__Sequence__init(controller__msg__ErrorMessage__Sequence * array, size_t size);

/// Finalize array of msg/ErrorMessage messages.
/**
 * It calls
 * controller__msg__ErrorMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
void
controller__msg__ErrorMessage__Sequence__fini(controller__msg__ErrorMessage__Sequence * array);

/// Create array of msg/ErrorMessage messages.
/**
 * It allocates the memory for the array and calls
 * controller__msg__ErrorMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
controller__msg__ErrorMessage__Sequence *
controller__msg__ErrorMessage__Sequence__create(size_t size);

/// Destroy array of msg/ErrorMessage messages.
/**
 * It calls
 * controller__msg__ErrorMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
void
controller__msg__ErrorMessage__Sequence__destroy(controller__msg__ErrorMessage__Sequence * array);

/// Check for msg/ErrorMessage message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_controller
bool
controller__msg__ErrorMessage__Sequence__are_equal(const controller__msg__ErrorMessage__Sequence * lhs, const controller__msg__ErrorMessage__Sequence * rhs);

/// Copy an array of msg/ErrorMessage messages.
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
controller__msg__ErrorMessage__Sequence__copy(
  const controller__msg__ErrorMessage__Sequence * input,
  controller__msg__ErrorMessage__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CONTROLLER__MSG__DETAIL__ERROR_MESSAGE__FUNCTIONS_H_
