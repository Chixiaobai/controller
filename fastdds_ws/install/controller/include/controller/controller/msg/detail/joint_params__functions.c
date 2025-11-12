// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/JointParams.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/joint_params__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
controller__msg__JointParams__init(controller__msg__JointParams * msg)
{
  if (!msg) {
    return false;
  }
  // joint_index
  // max_pos
  // min_pos
  // max_vel
  // max_acc
  return true;
}

void
controller__msg__JointParams__fini(controller__msg__JointParams * msg)
{
  if (!msg) {
    return;
  }
  // joint_index
  // max_pos
  // min_pos
  // max_vel
  // max_acc
}

bool
controller__msg__JointParams__are_equal(const controller__msg__JointParams * lhs, const controller__msg__JointParams * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint_index
  if (lhs->joint_index != rhs->joint_index) {
    return false;
  }
  // max_pos
  if (lhs->max_pos != rhs->max_pos) {
    return false;
  }
  // min_pos
  if (lhs->min_pos != rhs->min_pos) {
    return false;
  }
  // max_vel
  if (lhs->max_vel != rhs->max_vel) {
    return false;
  }
  // max_acc
  if (lhs->max_acc != rhs->max_acc) {
    return false;
  }
  return true;
}

bool
controller__msg__JointParams__copy(
  const controller__msg__JointParams * input,
  controller__msg__JointParams * output)
{
  if (!input || !output) {
    return false;
  }
  // joint_index
  output->joint_index = input->joint_index;
  // max_pos
  output->max_pos = input->max_pos;
  // min_pos
  output->min_pos = input->min_pos;
  // max_vel
  output->max_vel = input->max_vel;
  // max_acc
  output->max_acc = input->max_acc;
  return true;
}

controller__msg__JointParams *
controller__msg__JointParams__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__JointParams * msg = (controller__msg__JointParams *)allocator.allocate(sizeof(controller__msg__JointParams), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__JointParams));
  bool success = controller__msg__JointParams__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__JointParams__destroy(controller__msg__JointParams * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__JointParams__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__JointParams__Sequence__init(controller__msg__JointParams__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__JointParams * data = NULL;

  if (size) {
    data = (controller__msg__JointParams *)allocator.zero_allocate(size, sizeof(controller__msg__JointParams), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__JointParams__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__JointParams__fini(&data[i - 1]);
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
controller__msg__JointParams__Sequence__fini(controller__msg__JointParams__Sequence * array)
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
      controller__msg__JointParams__fini(&array->data[i]);
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

controller__msg__JointParams__Sequence *
controller__msg__JointParams__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__JointParams__Sequence * array = (controller__msg__JointParams__Sequence *)allocator.allocate(sizeof(controller__msg__JointParams__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__JointParams__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__JointParams__Sequence__destroy(controller__msg__JointParams__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__JointParams__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__JointParams__Sequence__are_equal(const controller__msg__JointParams__Sequence * lhs, const controller__msg__JointParams__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__JointParams__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__JointParams__Sequence__copy(
  const controller__msg__JointParams__Sequence * input,
  controller__msg__JointParams__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__JointParams);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__JointParams * data =
      (controller__msg__JointParams *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__JointParams__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__JointParams__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__JointParams__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
