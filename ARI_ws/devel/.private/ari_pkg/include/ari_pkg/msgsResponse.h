// Generated by gencpp from file ari_pkg/msgsResponse.msg
// DO NOT EDIT!


#ifndef ARI_PKG_MESSAGE_MSGSRESPONSE_H
#define ARI_PKG_MESSAGE_MSGSRESPONSE_H


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
struct msgsResponse_
{
  typedef msgsResponse_<ContainerAllocator> Type;

  msgsResponse_()
    : msg()  {
    }
  msgsResponse_(const ContainerAllocator& _alloc)
    : msg(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  _msg_type msg;





  typedef boost::shared_ptr< ::ari_pkg::msgsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ari_pkg::msgsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct msgsResponse_

typedef ::ari_pkg::msgsResponse_<std::allocator<void> > msgsResponse;

typedef boost::shared_ptr< ::ari_pkg::msgsResponse > msgsResponsePtr;
typedef boost::shared_ptr< ::ari_pkg::msgsResponse const> msgsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ari_pkg::msgsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ari_pkg::msgsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ari_pkg::msgsResponse_<ContainerAllocator1> & lhs, const ::ari_pkg::msgsResponse_<ContainerAllocator2> & rhs)
{
  return lhs.msg == rhs.msg;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ari_pkg::msgsResponse_<ContainerAllocator1> & lhs, const ::ari_pkg::msgsResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ari_pkg

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ari_pkg::msgsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ari_pkg::msgsResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ari_pkg::msgsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ari_pkg::msgsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ari_pkg::msgsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ari_pkg::msgsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ari_pkg::msgsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7d96ed730776804754140b85e64c862e";
  }

  static const char* value(const ::ari_pkg::msgsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7d96ed7307768047ULL;
  static const uint64_t static_value2 = 0x54140b85e64c862eULL;
};

template<class ContainerAllocator>
struct DataType< ::ari_pkg::msgsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ari_pkg/msgsResponse";
  }

  static const char* value(const ::ari_pkg::msgsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ari_pkg::msgsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string msg\n"
"\n"
;
  }

  static const char* value(const ::ari_pkg::msgsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ari_pkg::msgsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.msg);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct msgsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ari_pkg::msgsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ari_pkg::msgsResponse_<ContainerAllocator>& v)
  {
    s << indent << "msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.msg);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ARI_PKG_MESSAGE_MSGSRESPONSE_H