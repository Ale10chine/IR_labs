// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interfaces:msg/CstmInforobot.idl
// generated code does not contain a copyright notice

#include "interfaces/msg/detail/cstm_inforobot__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interfaces
const rosidl_type_hash_t *
interfaces__msg__CstmInforobot__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe2, 0xeb, 0xcf, 0x8d, 0x3c, 0xb4, 0x94, 0x99,
      0xba, 0x9c, 0x0e, 0xca, 0xee, 0xf6, 0x7d, 0xec,
      0x76, 0x5a, 0x04, 0xb0, 0x55, 0xc3, 0x30, 0x9e,
      0xf0, 0x3f, 0xdd, 0x40, 0x85, 0x78, 0xd2, 0xdf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char interfaces__msg__CstmInforobot__TYPE_NAME[] = "interfaces/msg/CstmInforobot";

// Define type names, field names, and default values
static char interfaces__msg__CstmInforobot__FIELD_NAME__room_id[] = "room_id";
static char interfaces__msg__CstmInforobot__FIELD_NAME__room_name[] = "room_name";
static char interfaces__msg__CstmInforobot__FIELD_NAME__battery_level[] = "battery_level";

static rosidl_runtime_c__type_description__Field interfaces__msg__CstmInforobot__FIELDS[] = {
  {
    {interfaces__msg__CstmInforobot__FIELD_NAME__room_id, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__CstmInforobot__FIELD_NAME__room_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__CstmInforobot__FIELD_NAME__battery_level, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interfaces__msg__CstmInforobot__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interfaces__msg__CstmInforobot__TYPE_NAME, 28, 28},
      {interfaces__msg__CstmInforobot__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 room_id\n"
  "string room_name\n"
  "float64 battery_level";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interfaces__msg__CstmInforobot__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interfaces__msg__CstmInforobot__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 52, 52},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interfaces__msg__CstmInforobot__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interfaces__msg__CstmInforobot__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
