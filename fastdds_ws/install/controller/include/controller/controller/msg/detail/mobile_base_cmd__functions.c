// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/MobileBaseCmd.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/mobile_base_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
controller__msg__MobileBaseCmd__init(controller__msg__MobileBaseCmd * msg)
{
  if (!msg) {
    return false;
  }
  // linear_velocity
  // angular_velocity
  // lifetime
  return true;
}

void
controller__msg__MobileBaseCmd__fini(controller__msg__MobileBaseCmd * msg)
{
  if (!msg) {
    return;
  }
  // linear_velocity
  // angular_velocity
  // lifetime
}

bool
controller__msg__MobileBaseCmd__are_equal(const controller__msg__MobileBaseCmd * lhs, const controller__msg__MobileBaseCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  // lifetime
  if (lhs->lifetime != rhs->lifetime) {
    return false;
  }
  return true;
}

bool
controller__msg__MobileBaseCmd__copy(
  const controller__msg__MobileBaseCmd * input,
  controller__msg__MobileBaseCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  // lifetime
  output->lifetime = input->lifetime;
  return true;
}

controller__msg__MobileBaseCmd *
controller__msg__MobileBaseCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__MobileBaseCmd * msg = (controller__msg__MobileBaseCmd *)allocator.allocate(sizeof(controller__msg__MobileBaseCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__MobileBaseCmd));
  bool success = controller__msg__MobileBaseCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__MobileBaseCmd__destroy(controller__msg__MobileBaseCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__MobileBaseCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__MobileBaseCmd__Sequence__init(controller__msg__MobileBaseCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__MobileBaseCmd * data = NULL;

  if (size) {
    data = (controller__msg__MobileBaseCmd *)allocator.zero_allocate(size, sizeof(controller__msg__MobileBaseCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__MobileBaseCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__MobileBaseCmd__fini(&data[i - 1]);
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
controller__msg__MobileBaseCmd__Sequence__fini(controller__msg__MobileBaseCmd__Sequence * array)
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
      controller__msg__MobileBaseCmd__fini(&array->data[i]);
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

controller__msg__MobileBaseCmd__Sequence *
controller__msg__MobileBaseCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__MobileBaseCmd__Sequence * array = (controller__msg__MobileBaseCmd__Sequence *)allocator.allocate(sizeof(controller__msg__MobileBaseCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__MobileBaseCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__MobileBaseCmd__Sequence__destroy(controller__msg__MobileBaseCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__MobileBaseCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__MobileBaseCmd__Sequence__are_equal(const controller__msg__MobileBaseCmd__Sequence * lhs, const controller__msg__MobileBaseCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__MobileBaseCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__MobileBaseCmd__Sequence__copy(
  const controller__msg__MobileBaseCmd__Sequence * input,
  controller__msg__MobileBaseCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__MobileBaseCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__MobileBaseCmd * data =
      (controller__msg__MobileBaseCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__MobileBaseCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__MobileBaseCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__MobileBaseCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
