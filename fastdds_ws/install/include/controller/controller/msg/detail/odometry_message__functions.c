// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/OdometryMessage.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/odometry_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
controller__msg__OdometryMessage__init(controller__msg__OdometryMessage * msg)
{
  if (!msg) {
    return false;
  }
  // accumulated_time
  // accumulated_mileage
  return true;
}

void
controller__msg__OdometryMessage__fini(controller__msg__OdometryMessage * msg)
{
  if (!msg) {
    return;
  }
  // accumulated_time
  // accumulated_mileage
}

bool
controller__msg__OdometryMessage__are_equal(const controller__msg__OdometryMessage * lhs, const controller__msg__OdometryMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // accumulated_time
  if (lhs->accumulated_time != rhs->accumulated_time) {
    return false;
  }
  // accumulated_mileage
  if (lhs->accumulated_mileage != rhs->accumulated_mileage) {
    return false;
  }
  return true;
}

bool
controller__msg__OdometryMessage__copy(
  const controller__msg__OdometryMessage * input,
  controller__msg__OdometryMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // accumulated_time
  output->accumulated_time = input->accumulated_time;
  // accumulated_mileage
  output->accumulated_mileage = input->accumulated_mileage;
  return true;
}

controller__msg__OdometryMessage *
controller__msg__OdometryMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__OdometryMessage * msg = (controller__msg__OdometryMessage *)allocator.allocate(sizeof(controller__msg__OdometryMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__OdometryMessage));
  bool success = controller__msg__OdometryMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__OdometryMessage__destroy(controller__msg__OdometryMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__OdometryMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__OdometryMessage__Sequence__init(controller__msg__OdometryMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__OdometryMessage * data = NULL;

  if (size) {
    data = (controller__msg__OdometryMessage *)allocator.zero_allocate(size, sizeof(controller__msg__OdometryMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__OdometryMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__OdometryMessage__fini(&data[i - 1]);
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
controller__msg__OdometryMessage__Sequence__fini(controller__msg__OdometryMessage__Sequence * array)
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
      controller__msg__OdometryMessage__fini(&array->data[i]);
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

controller__msg__OdometryMessage__Sequence *
controller__msg__OdometryMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__OdometryMessage__Sequence * array = (controller__msg__OdometryMessage__Sequence *)allocator.allocate(sizeof(controller__msg__OdometryMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__OdometryMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__OdometryMessage__Sequence__destroy(controller__msg__OdometryMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__OdometryMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__OdometryMessage__Sequence__are_equal(const controller__msg__OdometryMessage__Sequence * lhs, const controller__msg__OdometryMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__OdometryMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__OdometryMessage__Sequence__copy(
  const controller__msg__OdometryMessage__Sequence * input,
  controller__msg__OdometryMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__OdometryMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__OdometryMessage * data =
      (controller__msg__OdometryMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__OdometryMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__OdometryMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__OdometryMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
