// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hb_interfaces:msg/BotCmd.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD__STRUCT_HPP_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hb_interfaces__msg__BotCmd __attribute__((deprecated))
#else
# define DEPRECATED__hb_interfaces__msg__BotCmd __declspec(deprecated)
#endif

namespace hb_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BotCmd_
{
  using Type = BotCmd_<ContainerAllocator>;

  explicit BotCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->m1 = 0.0f;
      this->m2 = 0.0f;
      this->m3 = 0.0f;
      this->base = 0.0f;
      this->elbow = 0.0f;
    }
  }

  explicit BotCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->m1 = 0.0f;
      this->m2 = 0.0f;
      this->m3 = 0.0f;
      this->base = 0.0f;
      this->elbow = 0.0f;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _m1_type =
    float;
  _m1_type m1;
  using _m2_type =
    float;
  _m2_type m2;
  using _m3_type =
    float;
  _m3_type m3;
  using _base_type =
    float;
  _base_type base;
  using _elbow_type =
    float;
  _elbow_type elbow;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__m1(
    const float & _arg)
  {
    this->m1 = _arg;
    return *this;
  }
  Type & set__m2(
    const float & _arg)
  {
    this->m2 = _arg;
    return *this;
  }
  Type & set__m3(
    const float & _arg)
  {
    this->m3 = _arg;
    return *this;
  }
  Type & set__base(
    const float & _arg)
  {
    this->base = _arg;
    return *this;
  }
  Type & set__elbow(
    const float & _arg)
  {
    this->elbow = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hb_interfaces::msg::BotCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const hb_interfaces::msg::BotCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hb_interfaces::msg::BotCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hb_interfaces::msg::BotCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hb_interfaces::msg::BotCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hb_interfaces::msg::BotCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hb_interfaces::msg::BotCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hb_interfaces::msg::BotCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hb_interfaces::msg::BotCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hb_interfaces::msg::BotCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hb_interfaces__msg__BotCmd
    std::shared_ptr<hb_interfaces::msg::BotCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hb_interfaces__msg__BotCmd
    std::shared_ptr<hb_interfaces::msg::BotCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BotCmd_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->m1 != other.m1) {
      return false;
    }
    if (this->m2 != other.m2) {
      return false;
    }
    if (this->m3 != other.m3) {
      return false;
    }
    if (this->base != other.base) {
      return false;
    }
    if (this->elbow != other.elbow) {
      return false;
    }
    return true;
  }
  bool operator!=(const BotCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BotCmd_

// alias to use template instance with default allocator
using BotCmd =
  hb_interfaces::msg::BotCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hb_interfaces

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD__STRUCT_HPP_
