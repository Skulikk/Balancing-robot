// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sensors_pkg:msg/EncoderData.idl
// generated code does not contain a copyright notice
#include "sensors_pkg/msg/detail/encoder_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
sensors_pkg__msg__EncoderData__init(sensors_pkg__msg__EncoderData * msg)
{
  if (!msg) {
    return false;
  }
  // rpm1
  // rpm2
  return true;
}

void
sensors_pkg__msg__EncoderData__fini(sensors_pkg__msg__EncoderData * msg)
{
  if (!msg) {
    return;
  }
  // rpm1
  // rpm2
}

bool
sensors_pkg__msg__EncoderData__are_equal(const sensors_pkg__msg__EncoderData * lhs, const sensors_pkg__msg__EncoderData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rpm1
  if (lhs->rpm1 != rhs->rpm1) {
    return false;
  }
  // rpm2
  if (lhs->rpm2 != rhs->rpm2) {
    return false;
  }
  return true;
}

bool
sensors_pkg__msg__EncoderData__copy(
  const sensors_pkg__msg__EncoderData * input,
  sensors_pkg__msg__EncoderData * output)
{
  if (!input || !output) {
    return false;
  }
  // rpm1
  output->rpm1 = input->rpm1;
  // rpm2
  output->rpm2 = input->rpm2;
  return true;
}

sensors_pkg__msg__EncoderData *
sensors_pkg__msg__EncoderData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sensors_pkg__msg__EncoderData * msg = (sensors_pkg__msg__EncoderData *)allocator.allocate(sizeof(sensors_pkg__msg__EncoderData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sensors_pkg__msg__EncoderData));
  bool success = sensors_pkg__msg__EncoderData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sensors_pkg__msg__EncoderData__destroy(sensors_pkg__msg__EncoderData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sensors_pkg__msg__EncoderData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sensors_pkg__msg__EncoderData__Sequence__init(sensors_pkg__msg__EncoderData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sensors_pkg__msg__EncoderData * data = NULL;

  if (size) {
    data = (sensors_pkg__msg__EncoderData *)allocator.zero_allocate(size, sizeof(sensors_pkg__msg__EncoderData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sensors_pkg__msg__EncoderData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sensors_pkg__msg__EncoderData__fini(&data[i - 1]);
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
sensors_pkg__msg__EncoderData__Sequence__fini(sensors_pkg__msg__EncoderData__Sequence * array)
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
      sensors_pkg__msg__EncoderData__fini(&array->data[i]);
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

sensors_pkg__msg__EncoderData__Sequence *
sensors_pkg__msg__EncoderData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sensors_pkg__msg__EncoderData__Sequence * array = (sensors_pkg__msg__EncoderData__Sequence *)allocator.allocate(sizeof(sensors_pkg__msg__EncoderData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sensors_pkg__msg__EncoderData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sensors_pkg__msg__EncoderData__Sequence__destroy(sensors_pkg__msg__EncoderData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sensors_pkg__msg__EncoderData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sensors_pkg__msg__EncoderData__Sequence__are_equal(const sensors_pkg__msg__EncoderData__Sequence * lhs, const sensors_pkg__msg__EncoderData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sensors_pkg__msg__EncoderData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sensors_pkg__msg__EncoderData__Sequence__copy(
  const sensors_pkg__msg__EncoderData__Sequence * input,
  sensors_pkg__msg__EncoderData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sensors_pkg__msg__EncoderData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    sensors_pkg__msg__EncoderData * data =
      (sensors_pkg__msg__EncoderData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sensors_pkg__msg__EncoderData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          sensors_pkg__msg__EncoderData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sensors_pkg__msg__EncoderData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
