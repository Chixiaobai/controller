// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/ErrorMessage.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/error_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `msg`
#include "rosidl_runtime_c/string_functions.h"

bool
controller__msg__ErrorMessage__init(controller__msg__ErrorMessage * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // level
  // module
  // error_code
  // msg
  if (!rosidl_runtime_c__String__init(&msg->msg)) {
    controller__msg__ErrorMessage__fini(msg);
    return false;
  }
  return true;
}

void
controller__msg__ErrorMessage__fini(controller__msg__ErrorMessage * msg)
{
  if (!msg) {
    return;
  }
  // id
  // level
  // module
  // error_code
  // msg
  rosidl_runtime_c__String__fini(&msg->msg);
}

bool
controller__msg__ErrorMessage__are_equal(const controller__msg__ErrorMessage * lhs, const controller__msg__ErrorMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // level
  if (lhs->level != rhs->level) {
    return false;
  }
  // module
  if (lhs->module != rhs->module) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // msg
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->msg), &(rhs->msg)))
  {
    return false;
  }
  return true;
}

bool
controller__msg__ErrorMessage__copy(
  const controller__msg__ErrorMessage * input,
  controller__msg__ErrorMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // level
  output->level = input->level;
  // module
  output->module = input->module;
  // error_code
  output->error_code = input->error_code;
  // msg
  if (!rosidl_runtime_c__String__copy(
      &(input->msg), &(output->msg)))
  {
    return false;
  }
  return true;
}

controller__msg__ErrorMessage *
controller__msg__ErrorMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__ErrorMessage * msg = (controller__msg__ErrorMessage *)allocator.allocate(sizeof(controller__msg__ErrorMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__ErrorMessage));
  bool success = controller__msg__ErrorMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__ErrorMessage__destroy(controller__msg__ErrorMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__ErrorMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__ErrorMessage__Sequence__init(controller__msg__ErrorMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__ErrorMessage * data = NULL;

  if (size) {
    data = (controller__msg__ErrorMessage *)allocator.zero_allocate(size, sizeof(controller__msg__ErrorMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__ErrorMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__ErrorMessage__fini(&data[i - 1]);
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
controller__msg__ErrorMessage__Sequence__fini(controller__msg__ErrorMessage__Sequence * array)
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
      controller__msg__ErrorMessage__fini(&array->data[i]);
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

controller__msg__ErrorMessage__Sequence *
controller__msg__ErrorMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__ErrorMessage__Sequence * array = (controller__msg__ErrorMessage__Sequence *)allocator.allocate(sizeof(controller__msg__ErrorMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__ErrorMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__ErrorMessage__Sequence__destroy(controller__msg__ErrorMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__ErrorMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__ErrorMessage__Sequence__are_equal(const controller__msg__ErrorMessage__Sequence * lhs, const controller__msg__ErrorMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__ErrorMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__ErrorMessage__Sequence__copy(
  const controller__msg__ErrorMessage__Sequence * input,
  controller__msg__ErrorMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__ErrorMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__ErrorMessage * data =
      (controller__msg__ErrorMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__ErrorMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__ErrorMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__ErrorMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
