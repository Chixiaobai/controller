// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/TcpPoseParams.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/tcp_pose_params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
controller__msg__TcpPoseParams__init(controller__msg__TcpPoseParams * msg)
{
  if (!msg) {
    return false;
  }
  // type
  // pose
  if (!rosidl_runtime_c__double__Sequence__init(&msg->pose, 0)) {
    controller__msg__TcpPoseParams__fini(msg);
    return false;
  }
  return true;
}

void
controller__msg__TcpPoseParams__fini(controller__msg__TcpPoseParams * msg)
{
  if (!msg) {
    return;
  }
  // type
  // pose
  rosidl_runtime_c__double__Sequence__fini(&msg->pose);
}

bool
controller__msg__TcpPoseParams__are_equal(const controller__msg__TcpPoseParams * lhs, const controller__msg__TcpPoseParams * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // pose
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
controller__msg__TcpPoseParams__copy(
  const controller__msg__TcpPoseParams * input,
  controller__msg__TcpPoseParams * output)
{
  if (!input || !output) {
    return false;
  }
  // type
  output->type = input->type;
  // pose
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

controller__msg__TcpPoseParams *
controller__msg__TcpPoseParams__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__TcpPoseParams * msg = (controller__msg__TcpPoseParams *)allocator.allocate(sizeof(controller__msg__TcpPoseParams), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__TcpPoseParams));
  bool success = controller__msg__TcpPoseParams__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__TcpPoseParams__destroy(controller__msg__TcpPoseParams * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__TcpPoseParams__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__TcpPoseParams__Sequence__init(controller__msg__TcpPoseParams__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__TcpPoseParams * data = NULL;

  if (size) {
    data = (controller__msg__TcpPoseParams *)allocator.zero_allocate(size, sizeof(controller__msg__TcpPoseParams), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__TcpPoseParams__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__TcpPoseParams__fini(&data[i - 1]);
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
controller__msg__TcpPoseParams__Sequence__fini(controller__msg__TcpPoseParams__Sequence * array)
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
      controller__msg__TcpPoseParams__fini(&array->data[i]);
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

controller__msg__TcpPoseParams__Sequence *
controller__msg__TcpPoseParams__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__TcpPoseParams__Sequence * array = (controller__msg__TcpPoseParams__Sequence *)allocator.allocate(sizeof(controller__msg__TcpPoseParams__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__TcpPoseParams__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__TcpPoseParams__Sequence__destroy(controller__msg__TcpPoseParams__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__TcpPoseParams__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__TcpPoseParams__Sequence__are_equal(const controller__msg__TcpPoseParams__Sequence * lhs, const controller__msg__TcpPoseParams__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__TcpPoseParams__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__TcpPoseParams__Sequence__copy(
  const controller__msg__TcpPoseParams__Sequence * input,
  controller__msg__TcpPoseParams__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__TcpPoseParams);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__TcpPoseParams * data =
      (controller__msg__TcpPoseParams *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__TcpPoseParams__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__TcpPoseParams__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__TcpPoseParams__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
