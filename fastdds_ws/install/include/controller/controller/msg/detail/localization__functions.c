// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/Localization.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/localization__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
controller__msg__Localization__init(controller__msg__Localization * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    controller__msg__Localization__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    controller__msg__Localization__fini(msg);
    return false;
  }
  // confidence
  // valid_flag
  // error_msgs
  // trajectory_id
  return true;
}

void
controller__msg__Localization__fini(controller__msg__Localization * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // confidence
  // valid_flag
  // error_msgs
  // trajectory_id
}

bool
controller__msg__Localization__are_equal(const controller__msg__Localization * lhs, const controller__msg__Localization * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // valid_flag
  if (lhs->valid_flag != rhs->valid_flag) {
    return false;
  }
  // error_msgs
  if (lhs->error_msgs != rhs->error_msgs) {
    return false;
  }
  // trajectory_id
  if (lhs->trajectory_id != rhs->trajectory_id) {
    return false;
  }
  return true;
}

bool
controller__msg__Localization__copy(
  const controller__msg__Localization * input,
  controller__msg__Localization * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // valid_flag
  output->valid_flag = input->valid_flag;
  // error_msgs
  output->error_msgs = input->error_msgs;
  // trajectory_id
  output->trajectory_id = input->trajectory_id;
  return true;
}

controller__msg__Localization *
controller__msg__Localization__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__Localization * msg = (controller__msg__Localization *)allocator.allocate(sizeof(controller__msg__Localization), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__Localization));
  bool success = controller__msg__Localization__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__Localization__destroy(controller__msg__Localization * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__Localization__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__Localization__Sequence__init(controller__msg__Localization__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__Localization * data = NULL;

  if (size) {
    data = (controller__msg__Localization *)allocator.zero_allocate(size, sizeof(controller__msg__Localization), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__Localization__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__Localization__fini(&data[i - 1]);
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
controller__msg__Localization__Sequence__fini(controller__msg__Localization__Sequence * array)
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
      controller__msg__Localization__fini(&array->data[i]);
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

controller__msg__Localization__Sequence *
controller__msg__Localization__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__Localization__Sequence * array = (controller__msg__Localization__Sequence *)allocator.allocate(sizeof(controller__msg__Localization__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__Localization__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__Localization__Sequence__destroy(controller__msg__Localization__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__Localization__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__Localization__Sequence__are_equal(const controller__msg__Localization__Sequence * lhs, const controller__msg__Localization__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__Localization__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__Localization__Sequence__copy(
  const controller__msg__Localization__Sequence * input,
  controller__msg__Localization__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__Localization);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__Localization * data =
      (controller__msg__Localization *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__Localization__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__Localization__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__Localization__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
