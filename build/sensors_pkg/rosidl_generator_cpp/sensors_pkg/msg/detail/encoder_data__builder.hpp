// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sensors_pkg:msg/EncoderData.idl
// generated code does not contain a copyright notice

#ifndef SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__BUILDER_HPP_
#define SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "sensors_pkg/msg/detail/encoder_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace sensors_pkg
{

namespace msg
{

namespace builder
{

class Init_EncoderData_rpm2
{
public:
  explicit Init_EncoderData_rpm2(::sensors_pkg::msg::EncoderData & msg)
  : msg_(msg)
  {}
  ::sensors_pkg::msg::EncoderData rpm2(::sensors_pkg::msg::EncoderData::_rpm2_type arg)
  {
    msg_.rpm2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sensors_pkg::msg::EncoderData msg_;
};

class Init_EncoderData_rpm1
{
public:
  Init_EncoderData_rpm1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EncoderData_rpm2 rpm1(::sensors_pkg::msg::EncoderData::_rpm1_type arg)
  {
    msg_.rpm1 = std::move(arg);
    return Init_EncoderData_rpm2(msg_);
  }

private:
  ::sensors_pkg::msg::EncoderData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sensors_pkg::msg::EncoderData>()
{
  return sensors_pkg::msg::builder::Init_EncoderData_rpm1();
}

}  // namespace sensors_pkg

#endif  // SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__BUILDER_HPP_
