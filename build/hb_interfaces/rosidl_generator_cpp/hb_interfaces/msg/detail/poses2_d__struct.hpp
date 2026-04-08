// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hb_interfaces:msg/Poses2D.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__POSES2_D__STRUCT_HPP_
#define HB_INTERFACES__MSG__DETAIL__POSES2_D__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'poses'
#include "hb_interfaces/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hb_interfaces__msg__Poses2D __attribute__((deprecated))
#else
# define DEPRECATED__hb_interfaces__msg__Poses2D __declspec(deprecated)
#endif

namespace hb_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Poses2D_
{
  using Type = Poses2D_<ContainerAllocator>;

  explicit Poses2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Poses2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _poses_type =
    std::vector<hb_interfaces::msg::Pose2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hb_interfaces::msg::Pose2D_<ContainerAllocator>>>;
  _poses_type poses;

  // setters for named parameter idiom
  Type & set__poses(
    const std::vector<hb_interfaces::msg::Pose2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hb_interfaces::msg::Pose2D_<ContainerAllocator>>> & _arg)
  {
    this->poses = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hb_interfaces::msg::Poses2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const hb_interfaces::msg::Poses2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hb_interfaces::msg::Poses2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hb_interfaces::msg::Poses2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hb_interfaces::msg::Poses2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hb_interfaces::msg::Poses2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hb_interfaces::msg::Poses2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hb_interfaces::msg::Poses2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hb_interfaces::msg::Poses2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hb_interfaces::msg::Poses2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hb_interfaces__msg__Poses2D
    std::shared_ptr<hb_interfaces::msg::Poses2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hb_interfaces__msg__Poses2D
    std::shared_ptr<hb_interfaces::msg::Poses2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Poses2D_ & other) const
  {
    if (this->poses != other.poses) {
      return false;
    }
    return true;
  }
  bool operator!=(const Poses2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Poses2D_

// alias to use template instance with default allocator
using Poses2D =
  hb_interfaces::msg::Poses2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hb_interfaces

#endif  // HB_INTERFACES__MSG__DETAIL__POSES2_D__STRUCT_HPP_
