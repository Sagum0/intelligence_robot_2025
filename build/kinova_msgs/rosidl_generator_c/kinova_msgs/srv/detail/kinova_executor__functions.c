// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from kinova_msgs:srv/KinovaExecutor.idl
// generated code does not contain a copyright notice
#include "kinova_msgs/srv/detail/kinova_executor__functions.h"

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
kinova_msgs__srv__KinovaExecutor_Request__init(kinova_msgs__srv__KinovaExecutor_Request * msg)
{
  if (!msg) {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__init(&msg->frame)) {
    kinova_msgs__srv__KinovaExecutor_Request__fini(msg);
    return false;
  }
  // coordinate
  if (!rosidl_runtime_c__float__Sequence__init(&msg->coordinate, 0)) {
    kinova_msgs__srv__KinovaExecutor_Request__fini(msg);
    return false;
  }
  return true;
}

void
kinova_msgs__srv__KinovaExecutor_Request__fini(kinova_msgs__srv__KinovaExecutor_Request * msg)
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
kinova_msgs__srv__KinovaExecutor_Request__are_equal(const kinova_msgs__srv__KinovaExecutor_Request * lhs, const kinova_msgs__srv__KinovaExecutor_Request * rhs)
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
kinova_msgs__srv__KinovaExecutor_Request__copy(
  const kinova_msgs__srv__KinovaExecutor_Request * input,
  kinova_msgs__srv__KinovaExecutor_Request * output)
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

kinova_msgs__srv__KinovaExecutor_Request *
kinova_msgs__srv__KinovaExecutor_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__srv__KinovaExecutor_Request * msg = (kinova_msgs__srv__KinovaExecutor_Request *)allocator.allocate(sizeof(kinova_msgs__srv__KinovaExecutor_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(kinova_msgs__srv__KinovaExecutor_Request));
  bool success = kinova_msgs__srv__KinovaExecutor_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
kinova_msgs__srv__KinovaExecutor_Request__destroy(kinova_msgs__srv__KinovaExecutor_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    kinova_msgs__srv__KinovaExecutor_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
kinova_msgs__srv__KinovaExecutor_Request__Sequence__init(kinova_msgs__srv__KinovaExecutor_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__srv__KinovaExecutor_Request * data = NULL;

  if (size) {
    data = (kinova_msgs__srv__KinovaExecutor_Request *)allocator.zero_allocate(size, sizeof(kinova_msgs__srv__KinovaExecutor_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = kinova_msgs__srv__KinovaExecutor_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        kinova_msgs__srv__KinovaExecutor_Request__fini(&data[i - 1]);
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
kinova_msgs__srv__KinovaExecutor_Request__Sequence__fini(kinova_msgs__srv__KinovaExecutor_Request__Sequence * array)
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
      kinova_msgs__srv__KinovaExecutor_Request__fini(&array->data[i]);
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

kinova_msgs__srv__KinovaExecutor_Request__Sequence *
kinova_msgs__srv__KinovaExecutor_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__srv__KinovaExecutor_Request__Sequence * array = (kinova_msgs__srv__KinovaExecutor_Request__Sequence *)allocator.allocate(sizeof(kinova_msgs__srv__KinovaExecutor_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = kinova_msgs__srv__KinovaExecutor_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
kinova_msgs__srv__KinovaExecutor_Request__Sequence__destroy(kinova_msgs__srv__KinovaExecutor_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    kinova_msgs__srv__KinovaExecutor_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
kinova_msgs__srv__KinovaExecutor_Request__Sequence__are_equal(const kinova_msgs__srv__KinovaExecutor_Request__Sequence * lhs, const kinova_msgs__srv__KinovaExecutor_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!kinova_msgs__srv__KinovaExecutor_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
kinova_msgs__srv__KinovaExecutor_Request__Sequence__copy(
  const kinova_msgs__srv__KinovaExecutor_Request__Sequence * input,
  kinova_msgs__srv__KinovaExecutor_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(kinova_msgs__srv__KinovaExecutor_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    kinova_msgs__srv__KinovaExecutor_Request * data =
      (kinova_msgs__srv__KinovaExecutor_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!kinova_msgs__srv__KinovaExecutor_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          kinova_msgs__srv__KinovaExecutor_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!kinova_msgs__srv__KinovaExecutor_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
kinova_msgs__srv__KinovaExecutor_Response__init(kinova_msgs__srv__KinovaExecutor_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    kinova_msgs__srv__KinovaExecutor_Response__fini(msg);
    return false;
  }
  return true;
}

void
kinova_msgs__srv__KinovaExecutor_Response__fini(kinova_msgs__srv__KinovaExecutor_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
kinova_msgs__srv__KinovaExecutor_Response__are_equal(const kinova_msgs__srv__KinovaExecutor_Response * lhs, const kinova_msgs__srv__KinovaExecutor_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
kinova_msgs__srv__KinovaExecutor_Response__copy(
  const kinova_msgs__srv__KinovaExecutor_Response * input,
  kinova_msgs__srv__KinovaExecutor_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

kinova_msgs__srv__KinovaExecutor_Response *
kinova_msgs__srv__KinovaExecutor_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__srv__KinovaExecutor_Response * msg = (kinova_msgs__srv__KinovaExecutor_Response *)allocator.allocate(sizeof(kinova_msgs__srv__KinovaExecutor_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(kinova_msgs__srv__KinovaExecutor_Response));
  bool success = kinova_msgs__srv__KinovaExecutor_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
kinova_msgs__srv__KinovaExecutor_Response__destroy(kinova_msgs__srv__KinovaExecutor_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    kinova_msgs__srv__KinovaExecutor_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
kinova_msgs__srv__KinovaExecutor_Response__Sequence__init(kinova_msgs__srv__KinovaExecutor_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__srv__KinovaExecutor_Response * data = NULL;

  if (size) {
    data = (kinova_msgs__srv__KinovaExecutor_Response *)allocator.zero_allocate(size, sizeof(kinova_msgs__srv__KinovaExecutor_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = kinova_msgs__srv__KinovaExecutor_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        kinova_msgs__srv__KinovaExecutor_Response__fini(&data[i - 1]);
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
kinova_msgs__srv__KinovaExecutor_Response__Sequence__fini(kinova_msgs__srv__KinovaExecutor_Response__Sequence * array)
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
      kinova_msgs__srv__KinovaExecutor_Response__fini(&array->data[i]);
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

kinova_msgs__srv__KinovaExecutor_Response__Sequence *
kinova_msgs__srv__KinovaExecutor_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  kinova_msgs__srv__KinovaExecutor_Response__Sequence * array = (kinova_msgs__srv__KinovaExecutor_Response__Sequence *)allocator.allocate(sizeof(kinova_msgs__srv__KinovaExecutor_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = kinova_msgs__srv__KinovaExecutor_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
kinova_msgs__srv__KinovaExecutor_Response__Sequence__destroy(kinova_msgs__srv__KinovaExecutor_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    kinova_msgs__srv__KinovaExecutor_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
kinova_msgs__srv__KinovaExecutor_Response__Sequence__are_equal(const kinova_msgs__srv__KinovaExecutor_Response__Sequence * lhs, const kinova_msgs__srv__KinovaExecutor_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!kinova_msgs__srv__KinovaExecutor_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
kinova_msgs__srv__KinovaExecutor_Response__Sequence__copy(
  const kinova_msgs__srv__KinovaExecutor_Response__Sequence * input,
  kinova_msgs__srv__KinovaExecutor_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(kinova_msgs__srv__KinovaExecutor_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    kinova_msgs__srv__KinovaExecutor_Response * data =
      (kinova_msgs__srv__KinovaExecutor_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!kinova_msgs__srv__KinovaExecutor_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          kinova_msgs__srv__KinovaExecutor_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!kinova_msgs__srv__KinovaExecutor_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
