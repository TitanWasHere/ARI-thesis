// Generated by gencpp from file ari_pkg/msgsRequest.msg
// DO NOT EDIT!


#ifndef ARI_PKG_MESSAGE_MSGSREQUEST_H
#define ARI_PKG_MESSAGE_MSGSREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ari_pkg
{
template <class ContainerAllocator>
struct msgsRequest_
{
  typedef msgsRequest_<ContainerAllocator> Type;

  msgsRequest_()
    : msg()  {
    }
  msgsRequest_(const ContainerAllocator& _alloc)
    : msg(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  _msg_type msg;





  typedef boost::shared_ptr< ::ari_pkg::msgsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ari_pkg::msgsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct msgsRequest_

typedef ::ari_pkg::msgsRequest_<std::allocator<void> > msgsRequest;

typedef boost::shared_ptr< ::ari_pkg::msgsRequest > msgsRequestPtr;
typedef boost::shared_ptr< ::ari_pkg::msgsRequest const> msgsRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ari_pkg::msgsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ari_pkg::msgsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ari_pkg::msgsRequest_<ContainerAllocator1> & lhs, const ::ari_pkg::msgsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.msg == rhs.msg;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ari_pkg::msgsRequest_<ContainerAllocator1> & lhs, const ::ari_pkg::msgsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ari_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ari_pkg::msgsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ari_pkg::msgsRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ari_pkg::msgsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ari_pkg::msgsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ari_pkg::msgsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ari_pkg::msgsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ari_pkg::msgsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7d96ed730776804754140b85e64c862e";
  }

  static const char* value(const ::ari_pkg::msgsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7d96ed7307768047ULL;
  static const uint64_t static_value2 = 0x54140b85e64c862eULL;
};

template<class ContainerAllocator>
struct DataType< ::ari_pkg::msgsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ari_pkg/msgsRequest";
  }

  static const char* value(const ::ari_pkg::msgsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ari_pkg::msgsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string msg\n"
;
  }

  static const char* value(const ::ari_pkg::msgsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ari_pkg::msgsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct msgsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ari_pkg::msgsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ari_pkg::msgsRequest_<ContainerAllocator>& v)
  {
    s << indent << "msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARI_PKG_MESSAGE_MSGSREQUEST_H