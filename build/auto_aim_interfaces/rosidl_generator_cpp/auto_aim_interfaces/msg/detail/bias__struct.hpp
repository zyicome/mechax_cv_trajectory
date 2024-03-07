// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from auto_aim_interfaces:msg/Bias.idl
// generated code does not contain a copyright notice

#ifndef AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__STRUCT_HPP_
#define AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__auto_aim_interfaces__msg__Bias __attribute__((deprecated))
#else
# define DEPRECATED__auto_aim_interfaces__msg__Bias __declspec(deprecated)
#endif

namespace auto_aim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Bias_
{
  using Type = Bias_<ContainerAllocator>;

  explicit Bias_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->needchangeyaw = 0.0;
      this->is_can_hit = false;
    }
  }

  explicit Bias_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->needchangeyaw = 0.0;
      this->is_can_hit = false;
    }
  }

  // field types and members
  using _needchangeyaw_type =
    double;
  _needchangeyaw_type needchangeyaw;
  using _is_can_hit_type =
    bool;
  _is_can_hit_type is_can_hit;

  // setters for named parameter idiom
  Type & set__needchangeyaw(
    const double & _arg)
  {
    this->needchangeyaw = _arg;
    return *this;
  }
  Type & set__is_can_hit(
    const bool & _arg)
  {
    this->is_can_hit = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    auto_aim_interfaces::msg::Bias_<ContainerAllocator> *;
  using ConstRawPtr =
    const auto_aim_interfaces::msg::Bias_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Bias_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<auto_aim_interfaces::msg::Bias_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Bias_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Bias_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      auto_aim_interfaces::msg::Bias_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<auto_aim_interfaces::msg::Bias_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Bias_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<auto_aim_interfaces::msg::Bias_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__auto_aim_interfaces__msg__Bias
    std::shared_ptr<auto_aim_interfaces::msg::Bias_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__auto_aim_interfaces__msg__Bias
    std::shared_ptr<auto_aim_interfaces::msg::Bias_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Bias_ & other) const
  {
    if (this->needchangeyaw != other.needchangeyaw) {
      return false;
    }
    if (this->is_can_hit != other.is_can_hit) {
      return false;
    }
    return true;
  }
  bool operator!=(const Bias_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Bias_

// alias to use template instance with default allocator
using Bias =
  auto_aim_interfaces::msg::Bias_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace auto_aim_interfaces

#endif  // AUTO_AIM_INTERFACES__MSG__DETAIL__BIAS__STRUCT_HPP_
