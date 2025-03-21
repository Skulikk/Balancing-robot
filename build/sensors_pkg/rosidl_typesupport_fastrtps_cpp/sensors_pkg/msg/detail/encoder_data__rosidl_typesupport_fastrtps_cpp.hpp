// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from sensors_pkg:msg/EncoderData.idl
// generated code does not contain a copyright notice

#ifndef SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "sensors_pkg/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "sensors_pkg/msg/detail/encoder_data__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace sensors_pkg
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sensors_pkg
cdr_serialize(
  const sensors_pkg::msg::EncoderData & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sensors_pkg
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sensors_pkg::msg::EncoderData & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sensors_pkg
get_serialized_size(
  const sensors_pkg::msg::EncoderData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sensors_pkg
max_serialized_size_EncoderData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace sensors_pkg

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sensors_pkg
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sensors_pkg, msg, EncoderData)();

#ifdef __cplusplus
}
#endif

#endif  // SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
