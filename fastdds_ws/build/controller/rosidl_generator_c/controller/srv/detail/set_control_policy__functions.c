// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from controller:srv/SetControlPolicy.idl
// generated code does not contain a copyright notice
#include "controller/srv/detail/set_control_policy__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
controller__srv__SetControlPolicy_Request__init(controller__srv__SetControlPolicy_Request * msg)
{
  if (!msg) {
    return false;
  }
  // policy
  return true;
}

void
controller__srv__SetControlPolicy_Request__fini(controller__srv__SetControlPolicy_Request * msg)
{
  if (!msg) {
    return;
  }
  // policy
}

bool
controller__srv__SetControlPolicy_Request__are_equal(const controller__srv__SetControlPolicy_Request * lhs, const controller__srv__SetControlPolicy_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // policy
  if (lhs->policy != rhs->policy) {
    return false;
  }
  return true;
}

bool
controller__srv__SetControlPolicy_Request__copy(
  const controller__srv__SetControlPolicy_Request * input,
  controller__srv__SetControlPolicy_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // policy
  output->policy = input->policy;
  return true;
}

controller__srv__SetControlPolicy_Request *
controller__srv__SetControlPolicy_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__SetControlPolicy_Request * msg = (controller__srv__SetControlPolicy_Request *)allocator.allocate(sizeof(controller__srv__SetControlPolicy_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__srv__SetControlPolicy_Request));
  bool success = controller__srv__SetControlPolicy_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__srv__SetControlPolicy_Request__destroy(controller__srv__SetControlPolicy_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__srv__SetControlPolicy_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__srv__SetControlPolicy_Request__Sequence__init(controller__srv__SetControlPolicy_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__SetControlPolicy_Request * data = NULL;

  if (size) {
    data = (controller__srv__SetControlPolicy_Request *)allocator.zero_allocate(size, sizeof(controller__srv__SetControlPolicy_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__srv__SetControlPolicy_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__srv__SetControlPolicy_Request__fini(&data[i - 1]);
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
controller__srv__SetControlPolicy_Request__Sequence__fini(controller__srv__SetControlPolicy_Request__Sequence * array)
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
      controller__srv__SetControlPolicy_Request__fini(&array->data[i]);
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

controller__srv__SetControlPolicy_Request__Sequence *
controller__srv__SetControlPolicy_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__SetControlPolicy_Request__Sequence * array = (controller__srv__SetControlPolicy_Request__Sequence *)allocator.allocate(sizeof(controller__srv__SetControlPolicy_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__srv__SetControlPolicy_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__srv__SetControlPolicy_Request__Sequence__destroy(controller__srv__SetControlPolicy_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__srv__SetControlPolicy_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__srv__SetControlPolicy_Request__Sequence__are_equal(const controller__srv__SetControlPolicy_Request__Sequence * lhs, const controller__srv__SetControlPolicy_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__srv__SetControlPolicy_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__srv__SetControlPolicy_Request__Sequence__copy(
  const controller__srv__SetControlPolicy_Request__Sequence * input,
  controller__srv__SetControlPolicy_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__srv__SetControlPolicy_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__srv__SetControlPolicy_Request * data =
      (controller__srv__SetControlPolicy_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__srv__SetControlPolicy_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__srv__SetControlPolicy_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__srv__SetControlPolicy_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
controller__srv__SetControlPolicy_Response__init(controller__srv__SetControlPolicy_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
controller__srv__SetControlPolicy_Response__fini(controller__srv__SetControlPolicy_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
controller__srv__SetControlPolicy_Response__are_equal(const controller__srv__SetControlPolicy_Response * lhs, const controller__srv__SetControlPolicy_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
controller__srv__SetControlPolicy_Response__copy(
  const controller__srv__SetControlPolicy_Response * input,
  controller__srv__SetControlPolicy_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

controller__srv__SetControlPolicy_Response *
controller__srv__SetControlPolicy_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__SetControlPolicy_Response * msg = (controller__srv__SetControlPolicy_Response *)allocator.allocate(sizeof(controller__srv__SetControlPolicy_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(controller__srv__SetControlPolicy_Response));
  bool success = controller__srv__SetControlPolicy_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
controller__srv__SetControlPolicy_Response__destroy(controller__srv__SetControlPolicy_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    controller__srv__SetControlPolicy_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
controller__srv__SetControlPolicy_Response__Sequence__init(controller__srv__SetControlPolicy_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__SetControlPolicy_Response * data = NULL;

  if (size) {
    data = (controller__srv__SetControlPolicy_Response *)allocator.zero_allocate(size, sizeof(controller__srv__SetControlPolicy_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = controller__srv__SetControlPolicy_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        controller__srv__SetControlPolicy_Response__fini(&data[i - 1]);
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
controller__srv__SetControlPolicy_Response__Sequence__fini(controller__srv__SetControlPolicy_Response__Sequence * array)
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
      controller__srv__SetControlPolicy_Response__fini(&array->data[i]);
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

controller__srv__SetControlPolicy_Response__Sequence *
controller__srv__SetControlPolicy_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  controller__srv__SetControlPolicy_Response__Sequence * array = (controller__srv__SetControlPolicy_Response__Sequence *)allocator.allocate(sizeof(controller__srv__SetControlPolicy_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = controller__srv__SetControlPolicy_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
controller__srv__SetControlPolicy_Response__Sequence__destroy(controller__srv__SetControlPolicy_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    controller__srv__SetControlPolicy_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
controller__srv__SetControlPolicy_Response__Sequence__are_equal(const controller__srv__SetControlPolicy_Response__Sequence * lhs, const controller__srv__SetControlPolicy_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!controller__srv__SetControlPolicy_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
controller__srv__SetControlPolicy_Response__Sequence__copy(
  const controller__srv__SetControlPolicy_Response__Sequence * input,
  controller__srv__SetControlPolicy_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(controller__srv__SetControlPolicy_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    controller__srv__SetControlPolicy_Response * data =
      (controller__srv__SetControlPolicy_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!controller__srv__SetControlPolicy_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          controller__srv__SetControlPolicy_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!controller__srv__SetControlPolicy_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
