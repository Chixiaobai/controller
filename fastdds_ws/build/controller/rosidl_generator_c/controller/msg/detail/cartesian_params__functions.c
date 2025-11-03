// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/CartesianParams.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/cartesian_params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
controller__msg__CartesianParams__init(controller__msg__CartesianParams * msg)
{
  if (!msg) {
    return false;
  }
  // cartesian_index
  // trans_max_vel
  // trans_max_acc
  // rota_max_vel
  // rota_max_acc
  return true;
}

void
controller__msg__CartesianParams__fini(controller__msg__CartesianParams * msg)
{
  if (!msg) {
    return;
  }
  // cartesian_index
  // trans_max_vel
  // trans_max_acc
  // rota_max_vel
  // rota_max_acc
}

bool
controller__msg__CartesianParams__are_equal(const controller__msg__CartesianParams * lhs, const controller__msg__CartesianParams * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cartesian_index
  if (lhs->cartesian_index != rhs->cartesian_index) {
    return false;
  }
  // trans_max_vel
  if (lhs->trans_max_vel != rhs->trans_max_vel) {
    return false;
  }
  // trans_max_acc
  if (lhs->trans_max_acc != rhs->trans_max_acc) {
    return false;
  }
  // rota_max_vel
  if (lhs->rota_max_vel != rhs->rota_max_vel) {
    return false;
  }
  // rota_max_acc
  if (lhs->rota_max_acc != rhs->rota_max_acc) {
    return false;
  }
  return true;
}

bool
controller__msg__CartesianParams__copy(
  const controller__msg__CartesianParams * input,
  controller__msg__CartesianParams * output)
{
  if (!input || !output) {
    return false;
  }
  // cartesian_index
  output->cartesian_index = input->cartesian_index;
  // trans_max_vel
  output->trans_max_vel = input->trans_max_vel;
  // trans_max_acc
  output->trans_max_acc = input->trans_max_acc;
  // rota_max_vel
  output->rota_max_vel = input->rota_max_vel;
  // rota_max_acc
  output->rota_max_acc = input->rota_max_acc;
  return true;
}

controller__msg__CartesianParams *
controller__msg__CartesianParams__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__CartesianParams * msg = (controller__msg__CartesianParams *)allocator.allocate(sizeof(controller__msg__CartesianParams), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__CartesianParams));
  bool success = controller__msg__CartesianParams__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__CartesianParams__destroy(controller__msg__CartesianParams * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__CartesianParams__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__CartesianParams__Sequence__init(controller__msg__CartesianParams__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__CartesianParams * data = NULL;

  if (size) {
    data = (controller__msg__CartesianParams *)allocator.zero_allocate(size, sizeof(controller__msg__CartesianParams), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__CartesianParams__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__CartesianParams__fini(&data[i - 1]);
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
controller__msg__CartesianParams__Sequence__fini(controller__msg__CartesianParams__Sequence * array)
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
      controller__msg__CartesianParams__fini(&array->data[i]);
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

controller__msg__CartesianParams__Sequence *
controller__msg__CartesianParams__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__CartesianParams__Sequence * array = (controller__msg__CartesianParams__Sequence *)allocator.allocate(sizeof(controller__msg__CartesianParams__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__CartesianParams__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__CartesianParams__Sequence__destroy(controller__msg__CartesianParams__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__CartesianParams__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__CartesianParams__Sequence__are_equal(const controller__msg__CartesianParams__Sequence * lhs, const controller__msg__CartesianParams__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__CartesianParams__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__CartesianParams__Sequence__copy(
  const controller__msg__CartesianParams__Sequence * input,
  controller__msg__CartesianParams__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__CartesianParams);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__CartesianParams * data =
      (controller__msg__CartesianParams *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__CartesianParams__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__CartesianParams__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__CartesianParams__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
