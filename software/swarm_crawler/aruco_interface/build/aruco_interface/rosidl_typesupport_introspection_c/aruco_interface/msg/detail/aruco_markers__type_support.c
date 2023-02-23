// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aruco_interface:msg/ArucoMarkers.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aruco_interface/msg/detail/aruco_markers__rosidl_typesupport_introspection_c.h"
#include "aruco_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aruco_interface/msg/detail/aruco_markers__functions.h"
#include "aruco_interface/msg/detail/aruco_markers__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `marker_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `poses`
#include "geometry_msgs/msg/pose.h"
// Member `poses`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aruco_interface__msg__ArucoMarkers__init(message_memory);
}

void ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_fini_function(void * message_memory)
{
  aruco_interface__msg__ArucoMarkers__fini(message_memory);
}

size_t ArucoMarkers__rosidl_typesupport_introspection_c__size_function__Pose__poses(
  const void * untyped_member)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return member->size;
}

const void * ArucoMarkers__rosidl_typesupport_introspection_c__get_const_function__Pose__poses(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ArucoMarkers__rosidl_typesupport_introspection_c__get_function__Pose__poses(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

bool ArucoMarkers__rosidl_typesupport_introspection_c__resize_function__Pose__poses(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  geometry_msgs__msg__Pose__Sequence__fini(member);
  return geometry_msgs__msg__Pose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface__msg__ArucoMarkers, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "marker_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface__msg__ArucoMarkers, marker_ids),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interface__msg__ArucoMarkers, poses),  // bytes offset in struct
    NULL,  // default value
    ArucoMarkers__rosidl_typesupport_introspection_c__size_function__Pose__poses,  // size() function pointer
    ArucoMarkers__rosidl_typesupport_introspection_c__get_const_function__Pose__poses,  // get_const(index) function pointer
    ArucoMarkers__rosidl_typesupport_introspection_c__get_function__Pose__poses,  // get(index) function pointer
    ArucoMarkers__rosidl_typesupport_introspection_c__resize_function__Pose__poses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_members = {
  "aruco_interface__msg",  // message namespace
  "ArucoMarkers",  // message name
  3,  // number of fields
  sizeof(aruco_interface__msg__ArucoMarkers),
  ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_member_array,  // message members
  ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_init_function,  // function to initialize message memory (memory has to be allocated)
  ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_type_support_handle = {
  0,
  &ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aruco_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interface, msg, ArucoMarkers)() {
  ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_type_support_handle.typesupport_identifier) {
    ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
