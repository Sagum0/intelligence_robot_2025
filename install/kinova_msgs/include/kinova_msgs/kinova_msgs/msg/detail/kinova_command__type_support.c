// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from kinova_msgs:msg/KinovaCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "kinova_msgs/msg/detail/kinova_command__rosidl_typesupport_introspection_c.h"
#include "kinova_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "kinova_msgs/msg/detail/kinova_command__functions.h"
#include "kinova_msgs/msg/detail/kinova_command__struct.h"


// Include directives for member types
// Member `frame`
#include "rosidl_runtime_c/string_functions.h"
// Member `coordinate`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  kinova_msgs__msg__KinovaCommand__init(message_memory);
}

void kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_fini_function(void * message_memory)
{
  kinova_msgs__msg__KinovaCommand__fini(message_memory);
}

size_t kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__size_function__KinovaCommand__coordinate(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__get_const_function__KinovaCommand__coordinate(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__get_function__KinovaCommand__coordinate(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__fetch_function__KinovaCommand__coordinate(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__get_const_function__KinovaCommand__coordinate(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__assign_function__KinovaCommand__coordinate(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__get_function__KinovaCommand__coordinate(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__resize_function__KinovaCommand__coordinate(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_message_member_array[2] = {
  {
    "frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kinova_msgs__msg__KinovaCommand, frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "coordinate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(kinova_msgs__msg__KinovaCommand, coordinate),  // bytes offset in struct
    NULL,  // default value
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__size_function__KinovaCommand__coordinate,  // size() function pointer
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__get_const_function__KinovaCommand__coordinate,  // get_const(index) function pointer
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__get_function__KinovaCommand__coordinate,  // get(index) function pointer
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__fetch_function__KinovaCommand__coordinate,  // fetch(index, &value) function pointer
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__assign_function__KinovaCommand__coordinate,  // assign(index, value) function pointer
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__resize_function__KinovaCommand__coordinate  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_message_members = {
  "kinova_msgs__msg",  // message namespace
  "KinovaCommand",  // message name
  2,  // number of fields
  sizeof(kinova_msgs__msg__KinovaCommand),
  kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_message_member_array,  // message members
  kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_message_type_support_handle = {
  0,
  &kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_kinova_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, kinova_msgs, msg, KinovaCommand)() {
  if (!kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_message_type_support_handle.typesupport_identifier) {
    kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &kinova_msgs__msg__KinovaCommand__rosidl_typesupport_introspection_c__KinovaCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
