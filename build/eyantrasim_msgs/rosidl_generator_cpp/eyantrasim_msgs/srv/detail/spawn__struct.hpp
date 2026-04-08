// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from eyantrasim_msgs:srv/Spawn.idl
// generated code does not contain a copyright notice

#ifndef EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__STRUCT_HPP_
#define EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__eyantrasim_msgs__srv__Spawn_Request __attribute__((deprecated))
#else
# define DEPRECATED__eyantrasim_msgs__srv__Spawn_Request __declspec(deprecated)
#endif

namespace eyantrasim_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Spawn_Request_
{
  using Type = Spawn_Request_<ContainerAllocator>;

  explicit Spawn_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->theta = 0.0f;
      this->r = 0;
      this->g = 0;
      this->b = 0;
      this->width = 0;
      this->image_index = 0l;
      this->name = "";
    }
  }

  explicit Spawn_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->theta = 0.0f;
      this->r = 0;
      this->g = 0;
      this->b = 0;
      this->width = 0;
      this->image_index = 0l;
      this->name = "";
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _theta_type =
    float;
  _theta_type theta;
  using _r_type =
    uint8_t;
  _r_type r;
  using _g_type =
    uint8_t;
  _g_type g;
  using _b_type =
    uint8_t;
  _b_type b;
  using _width_type =
    uint8_t;
  _width_type width;
  using _image_index_type =
    int32_t;
  _image_index_type image_index;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;

  // setters for named parameter idiom
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
  Type & set__theta(
    const float & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__r(
    const uint8_t & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__g(
    const uint8_t & _arg)
  {
    this->g = _arg;
    return *this;
  }
  Type & set__b(
    const uint8_t & _arg)
  {
    this->b = _arg;
    return *this;
  }
  Type & set__width(
    const uint8_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__image_index(
    const int32_t & _arg)
  {
    this->image_index = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eyantrasim_msgs__srv__Spawn_Request
    std::shared_ptr<eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eyantrasim_msgs__srv__Spawn_Request
    std::shared_ptr<eyantrasim_msgs::srv::Spawn_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Spawn_Request_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->g != other.g) {
      return false;
    }
    if (this->b != other.b) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->image_index != other.image_index) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    return true;
  }
  bool operator!=(const Spawn_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Spawn_Request_

// alias to use template instance with default allocator
using Spawn_Request =
  eyantrasim_msgs::srv::Spawn_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace eyantrasim_msgs


#ifndef _WIN32
# define DEPRECATED__eyantrasim_msgs__srv__Spawn_Response __attribute__((deprecated))
#else
# define DEPRECATED__eyantrasim_msgs__srv__Spawn_Response __declspec(deprecated)
#endif

namespace eyantrasim_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct Spawn_Response_
{
  using Type = Spawn_Response_<ContainerAllocator>;

  explicit Spawn_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  explicit Spawn_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__eyantrasim_msgs__srv__Spawn_Response
    std::shared_ptr<eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__eyantrasim_msgs__srv__Spawn_Response
    std::shared_ptr<eyantrasim_msgs::srv::Spawn_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Spawn_Response_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    return true;
  }
  bool operator!=(const Spawn_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Spawn_Response_

// alias to use template instance with default allocator
using Spawn_Response =
  eyantrasim_msgs::srv::Spawn_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace eyantrasim_msgs

namespace eyantrasim_msgs
{

namespace srv
{

struct Spawn
{
  using Request = eyantrasim_msgs::srv::Spawn_Request;
  using Response = eyantrasim_msgs::srv::Spawn_Response;
};

}  // namespace srv

}  // namespace eyantrasim_msgs

#endif  // EYANTRASIM_MSGS__SRV__DETAIL__SPAWN__STRUCT_HPP_
