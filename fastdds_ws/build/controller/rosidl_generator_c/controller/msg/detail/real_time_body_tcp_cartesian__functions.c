// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:msg/RealTimeBodyTcpCartesian.idl
// generated code does not contain a copyright notice
#include "controller/msg/detail/real_time_body_tcp_cartesian__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
controller__msg__RealTimeBodyTcpCartesian__init(controller__msg__RealTimeBodyTcpCartesian * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // left_arm
  // left_arm_valid
  // right_arm
  // right_arm_valid
  // torso
  // torso_valid
  // time
  return true;
}

void
controller__msg__RealTimeBodyTcpCartesian__fini(controller__msg__RealTimeBodyTcpCartesian * msg)
{
  if (!msg) {
    return;
  }
  // id
  // left_arm
  // left_arm_valid
  // right_arm
  // right_arm_valid
  // torso
  // torso_valid
  // time
}

bool
controller__msg__RealTimeBodyTcpCartesian__are_equal(const controller__msg__RealTimeBodyTcpCartesian * lhs, const controller__msg__RealTimeBodyTcpCartesian * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // left_arm
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->left_arm[i] != rhs->left_arm[i]) {
      return false;
    }
  }
  // left_arm_valid
  if (lhs->left_arm_valid != rhs->left_arm_valid) {
    return false;
  }
  // right_arm
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->right_arm[i] != rhs->right_arm[i]) {
      return false;
    }
  }
  // right_arm_valid
  if (lhs->right_arm_valid != rhs->right_arm_valid) {
    return false;
  }
  // torso
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->torso[i] != rhs->torso[i]) {
      return false;
    }
  }
  // torso_valid
  if (lhs->torso_valid != rhs->torso_valid) {
    return false;
  }
  // time
  if (lhs->time != rhs->time) {
    return false;
  }
  return true;
}

bool
controller__msg__RealTimeBodyTcpCartesian__copy(
  const controller__msg__RealTimeBodyTcpCartesian * input,
  controller__msg__RealTimeBodyTcpCartesian * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // left_arm
  for (size_t i = 0; i < 6; ++i) {
    output->left_arm[i] = input->left_arm[i];
  }
  // left_arm_valid
  output->left_arm_valid = input->left_arm_valid;
  // right_arm
  for (size_t i = 0; i < 6; ++i) {
    output->right_arm[i] = input->right_arm[i];
  }
  // right_arm_valid
  output->right_arm_valid = input->right_arm_valid;
  // torso
  for (size_t i = 0; i < 6; ++i) {
    output->torso[i] = input->torso[i];
  }
  // torso_valid
  output->torso_valid = input->torso_valid;
  // time
  output->time = input->time;
  return true;
}

controller__msg__RealTimeBodyTcpCartesian *
controller__msg__RealTimeBodyTcpCartesian__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__RealTimeBodyTcpCartesian * msg = (controller__msg__RealTimeBodyTcpCartesian *)allocator.allocate(sizeof(controller__msg__RealTimeBodyTcpCartesian), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__msg__RealTimeBodyTcpCartesian));
  bool success = controller__msg__RealTimeBodyTcpCartesian__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__msg__RealTimeBodyTcpCartesian__destroy(controller__msg__RealTimeBodyTcpCartesian * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__msg__RealTimeBodyTcpCartesian__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__msg__RealTimeBodyTcpCartesian__Sequence__init(controller__msg__RealTimeBodyTcpCartesian__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__RealTimeBodyTcpCartesian * data = NULL;

  if (size) {
    data = (controller__msg__RealTimeBodyTcpCartesian *)allocator.zero_allocate(size, sizeof(controller__msg__RealTimeBodyTcpCartesian), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__msg__RealTimeBodyTcpCartesian__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__msg__RealTimeBodyTcpCartesian__fini(&data[i - 1]);
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
controller__msg__RealTimeBodyTcpCartesian__Sequence__fini(controller__msg__RealTimeBodyTcpCartesian__Sequence * array)
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
      controller__msg__RealTimeBodyTcpCartesian__fini(&array->data[i]);
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

controller__msg__RealTimeBodyTcpCartesian__Sequence *
controller__msg__RealTimeBodyTcpCartesian__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__msg__RealTimeBodyTcpCartesian__Sequence * array = (controller__msg__RealTimeBodyTcpCartesian__Sequence *)allocator.allocate(sizeof(controller__msg__RealTimeBodyTcpCartesian__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__msg__RealTimeBodyTcpCartesian__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__msg__RealTimeBodyTcpCartesian__Sequence__destroy(controller__msg__RealTimeBodyTcpCartesian__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__msg__RealTimeBodyTcpCartesian__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__msg__RealTimeBodyTcpCartesian__Sequence__are_equal(const controller__msg__RealTimeBodyTcpCartesian__Sequence * lhs, const controller__msg__RealTimeBodyTcpCartesian__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__msg__RealTimeBodyTcpCartesian__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__msg__RealTimeBodyTcpCartesian__Sequence__copy(
  const controller__msg__RealTimeBodyTcpCartesian__Sequence * input,
  controller__msg__RealTimeBodyTcpCartesian__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__msg__RealTimeBodyTcpCartesian);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__msg__RealTimeBodyTcpCartesian * data =
      (controller__msg__RealTimeBodyTcpCartesian *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__msg__RealTimeBodyTcpCartesian__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__msg__RealTimeBodyTcpCartesian__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__msg__RealTimeBodyTcpCartesian__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
