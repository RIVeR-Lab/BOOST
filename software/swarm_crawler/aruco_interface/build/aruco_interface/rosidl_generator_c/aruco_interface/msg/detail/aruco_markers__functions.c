// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aruco_interface:msg/ArucoMarkers.idl
// generated code does not contain a copyright notice
#include "aruco_interface/msg/detail/aruco_markers__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `marker_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `poses`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
aruco_interface__msg__ArucoMarkers__init(aruco_interface__msg__ArucoMarkers * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    aruco_interface__msg__ArucoMarkers__fini(msg);
    return false;
  }
  // marker_ids
  if (!rosidl_runtime_c__int64__Sequence__init(&msg->marker_ids, 0)) {
    aruco_interface__msg__ArucoMarkers__fini(msg);
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__init(&msg->poses, 0)) {
    aruco_interface__msg__ArucoMarkers__fini(msg);
    return false;
  }
  return true;
}

void
aruco_interface__msg__ArucoMarkers__fini(aruco_interface__msg__ArucoMarkers * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // marker_ids
  rosidl_runtime_c__int64__Sequence__fini(&msg->marker_ids);
  // poses
  geometry_msgs__msg__Pose__Sequence__fini(&msg->poses);
}

bool
aruco_interface__msg__ArucoMarkers__are_equal(const aruco_interface__msg__ArucoMarkers * lhs, const aruco_interface__msg__ArucoMarkers * rhs)
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
  // marker_ids
  if (!rosidl_runtime_c__int64__Sequence__are_equal(
      &(lhs->marker_ids), &(rhs->marker_ids)))
  {
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__are_equal(
      &(lhs->poses), &(rhs->poses)))
  {
    return false;
  }
  return true;
}

bool
aruco_interface__msg__ArucoMarkers__copy(
  const aruco_interface__msg__ArucoMarkers * input,
  aruco_interface__msg__ArucoMarkers * output)
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
  // marker_ids
  if (!rosidl_runtime_c__int64__Sequence__copy(
      &(input->marker_ids), &(output->marker_ids)))
  {
    return false;
  }
  // poses
  if (!geometry_msgs__msg__Pose__Sequence__copy(
      &(input->poses), &(output->poses)))
  {
    return false;
  }
  return true;
}

aruco_interface__msg__ArucoMarkers *
aruco_interface__msg__ArucoMarkers__create()
{
  aruco_interface__msg__ArucoMarkers * msg = (aruco_interface__msg__ArucoMarkers *)malloc(sizeof(aruco_interface__msg__ArucoMarkers));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aruco_interface__msg__ArucoMarkers));
  bool success = aruco_interface__msg__ArucoMarkers__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
aruco_interface__msg__ArucoMarkers__destroy(aruco_interface__msg__ArucoMarkers * msg)
{
  if (msg) {
    aruco_interface__msg__ArucoMarkers__fini(msg);
  }
  free(msg);
}


bool
aruco_interface__msg__ArucoMarkers__Sequence__init(aruco_interface__msg__ArucoMarkers__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  aruco_interface__msg__ArucoMarkers * data = NULL;
  if (size) {
    data = (aruco_interface__msg__ArucoMarkers *)calloc(size, sizeof(aruco_interface__msg__ArucoMarkers));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aruco_interface__msg__ArucoMarkers__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aruco_interface__msg__ArucoMarkers__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
aruco_interface__msg__ArucoMarkers__Sequence__fini(aruco_interface__msg__ArucoMarkers__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      aruco_interface__msg__ArucoMarkers__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

aruco_interface__msg__ArucoMarkers__Sequence *
aruco_interface__msg__ArucoMarkers__Sequence__create(size_t size)
{
  aruco_interface__msg__ArucoMarkers__Sequence * array = (aruco_interface__msg__ArucoMarkers__Sequence *)malloc(sizeof(aruco_interface__msg__ArucoMarkers__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = aruco_interface__msg__ArucoMarkers__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
aruco_interface__msg__ArucoMarkers__Sequence__destroy(aruco_interface__msg__ArucoMarkers__Sequence * array)
{
  if (array) {
    aruco_interface__msg__ArucoMarkers__Sequence__fini(array);
  }
  free(array);
}

bool
aruco_interface__msg__ArucoMarkers__Sequence__are_equal(const aruco_interface__msg__ArucoMarkers__Sequence * lhs, const aruco_interface__msg__ArucoMarkers__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aruco_interface__msg__ArucoMarkers__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aruco_interface__msg__ArucoMarkers__Sequence__copy(
  const aruco_interface__msg__ArucoMarkers__Sequence * input,
  aruco_interface__msg__ArucoMarkers__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aruco_interface__msg__ArucoMarkers);
    aruco_interface__msg__ArucoMarkers * data =
      (aruco_interface__msg__ArucoMarkers *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aruco_interface__msg__ArucoMarkers__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          aruco_interface__msg__ArucoMarkers__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aruco_interface__msg__ArucoMarkers__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
