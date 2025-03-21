// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensors_pkg:msg/EncoderData.idl
// generated code does not contain a copyright notice

#ifndef SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__STRUCT_HPP_
#define SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__sensors_pkg__msg__EncoderData __attribute__((deprecated))
#else
# define DEPRECATED__sensors_pkg__msg__EncoderData __declspec(deprecated)
#endif

namespace sensors_pkg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EncoderData_
{
  using Type = EncoderData_<ContainerAllocator>;

  explicit EncoderData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rpm1 = 0.0f;
      this->rpm2 = 0.0f;
    }
  }

  explicit EncoderData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rpm1 = 0.0f;
      this->rpm2 = 0.0f;
    }
  }

  // field types and members
  using _rpm1_type =
    float;
  _rpm1_type rpm1;
  using _rpm2_type =
    float;
  _rpm2_type rpm2;

  // setters for named parameter idiom
  Type & set__rpm1(
    const float & _arg)
  {
    this->rpm1 = _arg;
    return *this;
  }
  Type & set__rpm2(
    const float & _arg)
  {
    this->rpm2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sensors_pkg::msg::EncoderData_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensors_pkg::msg::EncoderData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensors_pkg::msg::EncoderData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensors_pkg::msg::EncoderData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensors_pkg::msg::EncoderData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensors_pkg::msg::EncoderData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensors_pkg::msg::EncoderData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensors_pkg::msg::EncoderData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensors_pkg::msg::EncoderData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensors_pkg::msg::EncoderData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensors_pkg__msg__EncoderData
    std::shared_ptr<sensors_pkg::msg::EncoderData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensors_pkg__msg__EncoderData
    std::shared_ptr<sensors_pkg::msg::EncoderData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EncoderData_ & other) const
  {
    if (this->rpm1 != other.rpm1) {
      return false;
    }
    if (this->rpm2 != other.rpm2) {
      return false;
    }
    return true;
  }
  bool operator!=(const EncoderData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EncoderData_

// alias to use template instance with default allocator
using EncoderData =
  sensors_pkg::msg::EncoderData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sensors_pkg

#endif  // SENSORS_PKG__MSG__DETAIL__ENCODER_DATA__STRUCT_HPP_
