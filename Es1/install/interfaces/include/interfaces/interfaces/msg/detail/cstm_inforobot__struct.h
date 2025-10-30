// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/CstmInforobot.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/cstm_inforobot.h"


#ifndef INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__STRUCT_H_
#define INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'room_name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/CstmInforobot in the package interfaces.
typedef struct interfaces__msg__CstmInforobot
{
  int32_t room_id;
  rosidl_runtime_c__String room_name;
  double battery_level;
} interfaces__msg__CstmInforobot;

// Struct for a sequence of interfaces__msg__CstmInforobot.
typedef struct interfaces__msg__CstmInforobot__Sequence
{
  interfaces__msg__CstmInforobot * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__CstmInforobot__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__CSTM_INFOROBOT__STRUCT_H_
