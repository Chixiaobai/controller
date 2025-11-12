// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/JointAngle.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/joint_angle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
controller__msg__JointAngle__init(controller__msg__JointAngle * msg)
{
  if (!msg) {
    return false;
  }
  // joint_index
  // target_position
  // velocity
  return true;
}

void
controller__msg__JointAngle__fini(controller__msg__JointAngle * msg)
{
  if (!msg) {
    return;
  }
  // joint_index
  // target_position
  // velocity
}

bool
controller__msg__JointAngle__are_equal(const controller__msg__JointAngle * lhs, const controller__msg__JointAngle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint_index
  if (lhs->joint_index != rhs->joint_index) {
    return false;
  }
  // target_position
  if (lhs->target_position != rhs->target_position) {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  return true;
}

bool
controller__msg__JointAngle__copy(
  const controller__msg__JointAngle * input,
  controller__msg__JointAngle * output)
{
  if (!input || !output) {
    return false;
  }
  // joint_index
  output->joint_index = input->joint_index;
  // target_position
  output->target_position = input->target_position;
  // velocity
  output->velocity = input->velocity;
  return true;
}

controller__msg__JointAngle *
controller__msg__JointAngle__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__JointAngle * msg = (controller__msg__JointAngle *)allocator.allocate(sizeof(controller__msg__JointAngle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__JointAngle));
  bool success = controller__msg__JointAngle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__JointAngle__destroy(controller__msg__JointAngle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__JointAngle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__JointAngle__Sequence__init(controller__msg__JointAngle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__JointAngle * data = NULL;

  if (size) {
    data = (controller__msg__JointAngle *)allocator.zero_allocate(size, sizeof(controller__msg__JointAngle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__JointAngle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__JointAngle__fini(&data[i - 1]);
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
controller__msg__JointAngle__Sequence__fini(controller__msg__JointAngle__Sequence * array)
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
      controller__msg__JointAngle__fini(&array->data[i]);
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

controller__msg__JointAngle__Sequence *
controller__msg__JointAngle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__JointAngle__Sequence * array = (controller__msg__JointAngle__Sequence *)allocator.allocate(sizeof(controller__msg__JointAngle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__JointAngle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__JointAngle__Sequence__destroy(controller__msg__JointAngle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__JointAngle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__JointAngle__Sequence__are_equal(const controller__msg__JointAngle__Sequence * lhs, const controller__msg__JointAngle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__JointAngle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__JointAngle__Sequence__copy(
  const controller__msg__JointAngle__Sequence * input,
  controller__msg__JointAngle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__JointAngle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__JointAngle * data =
      (controller__msg__JointAngle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__JointAngle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__JointAngle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__JointAngle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
