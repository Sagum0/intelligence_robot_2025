// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from kinova_msgs:msg/KinovaCommand.idl
// generated code does not contain a copyright notice
#include "kinova_msgs/msg/detail/kinova_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `frame`
#include "rosidl_runtime_c/string_functions.h"
// Member `coordinate`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
kinova_msgs__msg__KinovaCommand__init(kinova_msgs__msg__KinovaCommand * msg)
{
  if (!msg) {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__init(&msg->frame)) {
    kinova_msgs__msg__KinovaCommand__fini(msg);
    return false;
  }
  // coordinate
  if (!rosidl_runtime_c__float__Sequence__init(&msg->coordinate, 0)) {
    kinova_msgs__msg__KinovaCommand__fini(msg);
    return false;
  }
  return true;
}

void
kinova_msgs__msg__KinovaCommand__fini(kinova_msgs__msg__KinovaCommand * msg)
{
  if (!msg) {
    return;
  }
  // frame
  rosidl_runtime_c__String__fini(&msg->frame);
  // coordinate
  rosidl_runtime_c__float__Sequence__fini(&msg->coordinate);
}

bool
kinova_msgs__msg__KinovaCommand__are_equal(const kinova_msgs__msg__KinovaCommand * lhs, const kinova_msgs__msg__KinovaCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->frame), &(rhs->frame)))
  {
    return false;
  }
  // coordinate
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->coordinate), &(rhs->coordinate)))
  {
    return false;
  }
  return true;
}

bool
kinova_msgs__msg__KinovaCommand__copy(
  const kinova_msgs__msg__KinovaCommand * input,
  kinova_msgs__msg__KinovaCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__copy(
      &(input->frame), &(output->frame)))
  {
    return false;
  }
  // coordinate
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->coordinate), &(output->coordinate)))
  {
    return false;
  }
  return true;
}

kinova_msgs__msg__KinovaCommand *
kinova_msgs__msg__KinovaCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__msg__KinovaCommand * msg = (kinova_msgs__msg__KinovaCommand *)allocator.allocate(sizeof(kinova_msgs__msg__KinovaCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(kinova_msgs__msg__KinovaCommand));
  bool success = kinova_msgs__msg__KinovaCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
kinova_msgs__msg__KinovaCommand__destroy(kinova_msgs__msg__KinovaCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    kinova_msgs__msg__KinovaCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
kinova_msgs__msg__KinovaCommand__Sequence__init(kinova_msgs__msg__KinovaCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__msg__KinovaCommand * data = NULL;

  if (size) {
    data = (kinova_msgs__msg__KinovaCommand *)allocator.zero_allocate(size, sizeof(kinova_msgs__msg__KinovaCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = kinova_msgs__msg__KinovaCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        kinova_msgs__msg__KinovaCommand__fini(&data[i - 1]);
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
kinova_msgs__msg__KinovaCommand__Sequence__fini(kinova_msgs__msg__KinovaCommand__Sequence * array)
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
      kinova_msgs__msg__KinovaCommand__fini(&array->data[i]);
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

kinova_msgs__msg__KinovaCommand__Sequence *
kinova_msgs__msg__KinovaCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__msg__KinovaCommand__Sequence * array = (kinova_msgs__msg__KinovaCommand__Sequence *)allocator.allocate(sizeof(kinova_msgs__msg__KinovaCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = kinova_msgs__msg__KinovaCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
kinova_msgs__msg__KinovaCommand__Sequence__destroy(kinova_msgs__msg__KinovaCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    kinova_msgs__msg__KinovaCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
kinova_msgs__msg__KinovaCommand__Sequence__are_equal(const kinova_msgs__msg__KinovaCommand__Sequence * lhs, const kinova_msgs__msg__KinovaCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!kinova_msgs__msg__KinovaCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
kinova_msgs__msg__KinovaCommand__Sequence__copy(
  const kinova_msgs__msg__KinovaCommand__Sequence * input,
  kinova_msgs__msg__KinovaCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(kinova_msgs__msg__KinovaCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    kinova_msgs__msg__KinovaCommand * data =
      (kinova_msgs__msg__KinovaCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!kinova_msgs__msg__KinovaCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          kinova_msgs__msg__KinovaCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!kinova_msgs__msg__KinovaCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
