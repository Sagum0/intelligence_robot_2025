// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kinova_msgs:msg/KinovaCommand.idl
// generated code does not contain a copyright notice

#ifndef KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__STRUCT_H_
#define KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'frame'
#include "rosidl_runtime_c/string.h"
// Member 'coordinate'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/KinovaCommand in the package kinova_msgs.
typedef struct kinova_msgs__msg__KinovaCommand
{
  rosidl_runtime_c__String frame;
  rosidl_runtime_c__float__Sequence coordinate;
} kinova_msgs__msg__KinovaCommand;

// Struct for a sequence of kinova_msgs__msg__KinovaCommand.
typedef struct kinova_msgs__msg__KinovaCommand__Sequence
{
  kinova_msgs__msg__KinovaCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kinova_msgs__msg__KinovaCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KINOVA_MSGS__MSG__DETAIL__KINOVA_COMMAND__STRUCT_H_
