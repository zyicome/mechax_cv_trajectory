// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/ReceiveSerial.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__RECEIVE_SERIAL__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__RECEIVE_SERIAL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__ReceiveSerial __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__ReceiveSerial __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReceiveSerial_
{
  using Type = ReceiveSerial_<ContainerAllocator>;

  explicit ReceiveSerial_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_shoot = false;
      this->yaw = 0.0;
      this->pitch = 0.0;
    }
  }

  explicit ReceiveSerial_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_shoot = false;
      this->yaw = 0.0;
      this->pitch = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _is_shoot_type =
    bool;
  _is_shoot_type is_shoot;
  using _yaw_type =
    double;
  _yaw_type yaw;
  using _pitch_type =
    double;
  _pitch_type pitch;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__is_shoot(
    const bool & _arg)
  {
    this->is_shoot = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__pitch(
    const double & _arg)
  {
    this->pitch = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__ReceiveSerial
    std::shared_ptr<auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__ReceiveSerial
    std::shared_ptr<auto_aim_interfaces::msg::ReceiveSerial_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReceiveSerial_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->is_shoot != other.is_shoot) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->pitch != other.pitch) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReceiveSerial_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReceiveSerial_

// alias to use template instance with default allocator
using ReceiveSerial =
  auto_aim_interfaces::msg::ReceiveSerial_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__RECEIVE_SERIAL__STRUCT_HPP_
