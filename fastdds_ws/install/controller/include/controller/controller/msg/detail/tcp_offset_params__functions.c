// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/TcpOffsetParams.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/tcp_offset_params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `offset`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
controller__msg__TcpOffsetParams__init(controller__msg__TcpOffsetParams * msg)
{
  if (!msg) {
    return false;
  }
  // type
  // offset
  if (!rosidl_runtime_c__double__Sequence__init(&msg->offset, 0)) {
    controller__msg__TcpOffsetParams__fini(msg);
    return false;
  }
  return true;
}

void
controller__msg__TcpOffsetParams__fini(controller__msg__TcpOffsetParams * msg)
{
  if (!msg) {
    return;
  }
  // type
  // offset
  rosidl_runtime_c__double__Sequence__fini(&msg->offset);
}

bool
controller__msg__TcpOffsetParams__are_equal(const controller__msg__TcpOffsetParams * lhs, const controller__msg__TcpOffsetParams * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // offset
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->offset), &(rhs->offset)))
  {
    return false;
  }
  return true;
}

bool
controller__msg__TcpOffsetParams__copy(
  const controller__msg__TcpOffsetParams * input,
  controller__msg__TcpOffsetParams * output)
{
  if (!input || !output) {
    return false;
  }
  // type
  output->type = input->type;
  // offset
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->offset), &(output->offset)))
  {
    return false;
  }
  return true;
}

controller__msg__TcpOffsetParams *
controller__msg__TcpOffsetParams__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__TcpOffsetParams * msg = (controller__msg__TcpOffsetParams *)allocator.allocate(sizeof(controller__msg__TcpOffsetParams), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__TcpOffsetParams));
  bool success = controller__msg__TcpOffsetParams__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__TcpOffsetParams__destroy(controller__msg__TcpOffsetParams * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__TcpOffsetParams__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__TcpOffsetParams__Sequence__init(controller__msg__TcpOffsetParams__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__TcpOffsetParams * data = NULL;

  if (size) {
    data = (controller__msg__TcpOffsetParams *)allocator.zero_allocate(size, sizeof(controller__msg__TcpOffsetParams), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__TcpOffsetParams__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__TcpOffsetParams__fini(&data[i - 1]);
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
controller__msg__TcpOffsetParams__Sequence__fini(controller__msg__TcpOffsetParams__Sequence * array)
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
      controller__msg__TcpOffsetParams__fini(&array->data[i]);
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

controller__msg__TcpOffsetParams__Sequence *
controller__msg__TcpOffsetParams__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__TcpOffsetParams__Sequence * array = (controller__msg__TcpOffsetParams__Sequence *)allocator.allocate(sizeof(controller__msg__TcpOffsetParams__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__TcpOffsetParams__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__TcpOffsetParams__Sequence__destroy(controller__msg__TcpOffsetParams__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__TcpOffsetParams__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__TcpOffsetParams__Sequence__are_equal(const controller__msg__TcpOffsetParams__Sequence * lhs, const controller__msg__TcpOffsetParams__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__TcpOffsetParams__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__TcpOffsetParams__Sequence__copy(
  const controller__msg__TcpOffsetParams__Sequence * input,
  controller__msg__TcpOffsetParams__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__TcpOffsetParams);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__TcpOffsetParams * data =
      (controller__msg__TcpOffsetParams *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__TcpOffsetParams__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__TcpOffsetParams__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__TcpOffsetParams__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
