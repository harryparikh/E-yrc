// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hb_interfaces:msg/BotCmdArray.idl
// generated code does not contain a copyright notice

#ifndef HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__STRUCT_HPP_
#define HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'cmds'
#include "hb_interfaces/msg/detail/bot_cmd__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hb_interfaces__msg__BotCmdArray __attribute__((deprecated))
#else
# define DEPRECATED__hb_interfaces__msg__BotCmdArray __declspec(deprecated)
#endif

namespace hb_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BotCmdArray_
{
  using Type = BotCmdArray_<ContainerAllocator>;

  explicit BotCmdArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BotCmdArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _cmds_type =
    std::vector<hb_interfaces::msg::BotCmd_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hb_interfaces::msg::BotCmd_<ContainerAllocator>>>;
  _cmds_type cmds;

  // setters for named parameter idiom
  Type & set__cmds(
    const std::vector<hb_interfaces::msg::BotCmd_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<hb_interfaces::msg::BotCmd_<ContainerAllocator>>> & _arg)
  {
    this->cmds = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hb_interfaces::msg::BotCmdArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const hb_interfaces::msg::BotCmdArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hb_interfaces::msg::BotCmdArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hb_interfaces::msg::BotCmdArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hb_interfaces::msg::BotCmdArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hb_interfaces::msg::BotCmdArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hb_interfaces::msg::BotCmdArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hb_interfaces::msg::BotCmdArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hb_interfaces::msg::BotCmdArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hb_interfaces::msg::BotCmdArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hb_interfaces__msg__BotCmdArray
    std::shared_ptr<hb_interfaces::msg::BotCmdArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hb_interfaces__msg__BotCmdArray
    std::shared_ptr<hb_interfaces::msg::BotCmdArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BotCmdArray_ & other) const
  {
    if (this->cmds != other.cmds) {
      return false;
    }
    return true;
  }
  bool operator!=(const BotCmdArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BotCmdArray_

// alias to use template instance with default allocator
using BotCmdArray =
  hb_interfaces::msg::BotCmdArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hb_interfaces

#endif  // HB_INTERFACES__MSG__DETAIL__BOT_CMD_ARRAY__STRUCT_HPP_
