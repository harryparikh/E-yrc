// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hb_interfaces:msg/Pose2D.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__POSE2_D__STRUCT_HPP_
#define HB_INTERFACES__MSG__DETAIL__POSE2_D__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__hb_interfaces__msg__Pose2D __attribute__((deprecated))
#else
# define DEPRECATED__hb_interfaces__msg__Pose2D __declspec(deprecated)
#endif

namespace hb_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Pose2D_
{
  using Type = Pose2D_<ContainerAllocator>;

  explicit Pose2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->x = 0.0f;
      this->y = 0.0f;
      this->w = 0.0f;
    }
  }

  explicit Pose2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->x = 0.0f;
      this->y = 0.0f;
      this->w = 0.0f;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _w_type =
    float;
  _w_type w;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__w(
    const float & _arg)
  {
    this->w = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hb_interfaces::msg::Pose2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const hb_interfaces::msg::Pose2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hb_interfaces::msg::Pose2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hb_interfaces::msg::Pose2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hb_interfaces::msg::Pose2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hb_interfaces::msg::Pose2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hb_interfaces::msg::Pose2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hb_interfaces::msg::Pose2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hb_interfaces::msg::Pose2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hb_interfaces::msg::Pose2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hb_interfaces__msg__Pose2D
    std::shared_ptr<hb_interfaces::msg::Pose2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hb_interfaces__msg__Pose2D
    std::shared_ptr<hb_interfaces::msg::Pose2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Pose2D_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->w != other.w) {
      return false;
    }
    return true;
  }
  bool operator!=(const Pose2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Pose2D_

// alias to use template instance with default allocator
using Pose2D =
  hb_interfaces::msg::Pose2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hb_interfaces

#endif  // HB_INTERFACES__MSG__DETAIL__POSE2_D__STRUCT_HPP_
