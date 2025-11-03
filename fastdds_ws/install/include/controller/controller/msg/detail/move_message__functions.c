// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/MoveMessage.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/move_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `position`
// Member `velocity`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `tcp_pose`
#include "controller/msg/detail/tcp_pose_params__functions.h"

bool
controller__msg__MoveMessage__init(controller__msg__MoveMessage * msg)
{
  if (!msg) {
    return false;
  }
  // state
  // position
  if (!rosidl_runtime_c__double__Sequence__init(&msg->position, 0)) {
    controller__msg__MoveMessage__fini(msg);
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__init(&msg->velocity, 0)) {
    controller__msg__MoveMessage__fini(msg);
    return false;
  }
  // tcp_pose
  if (!controller__msg__TcpPoseParams__Sequence__init(&msg->tcp_pose, 0)) {
    controller__msg__MoveMessage__fini(msg);
    return false;
  }
  // token
  return true;
}

void
controller__msg__MoveMessage__fini(controller__msg__MoveMessage * msg)
{
  if (!msg) {
    return;
  }
  // state
  // position
  rosidl_runtime_c__double__Sequence__fini(&msg->position);
  // velocity
  rosidl_runtime_c__double__Sequence__fini(&msg->velocity);
  // tcp_pose
  controller__msg__TcpPoseParams__Sequence__fini(&msg->tcp_pose);
  // token
}

bool
controller__msg__MoveMessage__are_equal(const controller__msg__MoveMessage * lhs, const controller__msg__MoveMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // state
  if (lhs->state != rhs->state) {
    return false;
  }
  // position
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // tcp_pose
  if (!controller__msg__TcpPoseParams__Sequence__are_equal(
      &(lhs->tcp_pose), &(rhs->tcp_pose)))
  {
    return false;
  }
  // token
  if (lhs->token != rhs->token) {
    return false;
  }
  return true;
}

bool
controller__msg__MoveMessage__copy(
  const controller__msg__MoveMessage * input,
  controller__msg__MoveMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // state
  output->state = input->state;
  // position
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // tcp_pose
  if (!controller__msg__TcpPoseParams__Sequence__copy(
      &(input->tcp_pose), &(output->tcp_pose)))
  {
    return false;
  }
  // token
  output->token = input->token;
  return true;
}

controller__msg__MoveMessage *
controller__msg__MoveMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__MoveMessage * msg = (controller__msg__MoveMessage *)allocator.allocate(sizeof(controller__msg__MoveMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__MoveMessage));
  bool success = controller__msg__MoveMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__MoveMessage__destroy(controller__msg__MoveMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__MoveMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__MoveMessage__Sequence__init(controller__msg__MoveMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__MoveMessage * data = NULL;

  if (size) {
    data = (controller__msg__MoveMessage *)allocator.zero_allocate(size, sizeof(controller__msg__MoveMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__MoveMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__MoveMessage__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
controller__msg__MoveMessage__Sequence__fini(controller__msg__MoveMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      controller__msg__MoveMessage__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

controller__msg__MoveMessage__Sequence *
controller__msg__MoveMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__MoveMessage__Sequence * array = (controller__msg__MoveMessage__Sequence *)allocator.allocate(sizeof(controller__msg__MoveMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__MoveMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__MoveMessage__Sequence__destroy(controller__msg__MoveMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__MoveMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__MoveMessage__Sequence__are_equal(const controller__msg__MoveMessage__Sequence * lhs, const controller__msg__MoveMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__MoveMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__MoveMessage__Sequence__copy(
  const controller__msg__MoveMessage__Sequence * input,
  controller__msg__MoveMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__MoveMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__MoveMessage * data =
      (controller__msg__MoveMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__MoveMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__MoveMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__MoveMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
