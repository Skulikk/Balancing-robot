// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sensors_pkg:msg/EncoderData.idl
// generated code does not contain a copyright notice

#ifndef SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__TRAITS_HPP_
#define SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "sensors_pkg/msg/detail/encoder_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace sensors_pkg
{

namespace msg
{

inline void to_flow_style_yaml(
  const EncoderData & msg,
  std::ostream & out)
{
  out << "{";
  // member: rpm1
  {
    out << "rpm1: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm1, out);
    out << ", ";
  }

  // member: rpm2
  {
    out << "rpm2: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EncoderData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rpm1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpm1: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm1, out);
    out << "\n";
  }

  // member: rpm2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpm2: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EncoderData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace sensors_pkg

namespace rosidl_generator_traits
{

[[deprecated("use sensors_pkg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const sensors_pkg::msg::EncoderData & msg,
  std::ostream & out, size_t indentation = 0)
{
  sensors_pkg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use sensors_pkg::msg::to_yaml() instead")]]
inline std::string to_yaml(const sensors_pkg::msg::EncoderData & msg)
{
  return sensors_pkg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<sensors_pkg::msg::EncoderData>()
{
  return "sensors_pkg::msg::EncoderData";
}

template<>
inline const char * name<sensors_pkg::msg::EncoderData>()
{
  return "sensors_pkg/msg/EncoderData";
}

template<>
struct has_fixed_size<sensors_pkg::msg::EncoderData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<sensors_pkg::msg::EncoderData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<sensors_pkg::msg::EncoderData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__TRAITS_HPP_
