// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sensors_pkg:msg/EncoderData.idl
// generated code does not contain a copyright notice

#ifndef SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__STRUCT_H_
#define SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/EncoderData in the package sensors_pkg.
typedef struct sensors_pkg__msg__EncoderData
{
  float rpm1;
  float rpm2;
} sensors_pkg__msg__EncoderData;

// Struct for a sequence of sensors_pkg__msg__EncoderData.
typedef struct sensors_pkg__msg__EncoderData__Sequence
{
  sensors_pkg__msg__EncoderData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sensors_pkg__msg__EncoderData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__STRUCT_H_
