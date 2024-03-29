/****************************************************************

  Generated by Eclipse Cyclone DDS IDL to CXX Translator
  File name: ExampleIdlData.idl
  Source: ExampleIdlData.hpp
  Cyclone DDS: v0.9.0

*****************************************************************/
#ifndef DDSCXX_EXAMPLEIDLDATA_HPP
#define DDSCXX_EXAMPLEIDLDATA_HPP

#include <cstdint>
#include <vector>
#include <string>

namespace ExampleIdlData
{
typedef std::vector<uint8_t> BytesArray;

class Msg
{
private:
 int32_t id_ = 0;
 std::string message_;
 ::ExampleIdlData::BytesArray payloadEigen_;
 ::ExampleIdlData::BytesArray payloadOpenCVImage_;
 ::ExampleIdlData::BytesArray payloadPCLPointCloud_;

public:
  Msg() = default;

  explicit Msg(
    int32_t id,
    const std::string& message,
    const ::ExampleIdlData::BytesArray& payloadEigen,
    const ::ExampleIdlData::BytesArray& payloadOpenCVImage,
    const ::ExampleIdlData::BytesArray& payloadPCLPointCloud) :
    id_(id),
    message_(message),
    payloadEigen_(payloadEigen),
    payloadOpenCVImage_(payloadOpenCVImage),
    payloadPCLPointCloud_(payloadPCLPointCloud) { }

  int32_t id() const { return this->id_; }
  int32_t& id() { return this->id_; }
  void id(int32_t _val_) { this->id_ = _val_; }
  const std::string& message() const { return this->message_; }
  std::string& message() { return this->message_; }
  void message(const std::string& _val_) { this->message_ = _val_; }
  void message(std::string&& _val_) { this->message_ = _val_; }
  const ::ExampleIdlData::BytesArray& payloadEigen() const { return this->payloadEigen_; }
  ::ExampleIdlData::BytesArray& payloadEigen() { return this->payloadEigen_; }
  void payloadEigen(const ::ExampleIdlData::BytesArray& _val_) { this->payloadEigen_ = _val_; }
  void payloadEigen(::ExampleIdlData::BytesArray&& _val_) { this->payloadEigen_ = _val_; }
  const ::ExampleIdlData::BytesArray& payloadOpenCVImage() const { return this->payloadOpenCVImage_; }
  ::ExampleIdlData::BytesArray& payloadOpenCVImage() { return this->payloadOpenCVImage_; }
  void payloadOpenCVImage(const ::ExampleIdlData::BytesArray& _val_) { this->payloadOpenCVImage_ = _val_; }
  void payloadOpenCVImage(::ExampleIdlData::BytesArray&& _val_) { this->payloadOpenCVImage_ = _val_; }
  const ::ExampleIdlData::BytesArray& payloadPCLPointCloud() const { return this->payloadPCLPointCloud_; }
  ::ExampleIdlData::BytesArray& payloadPCLPointCloud() { return this->payloadPCLPointCloud_; }
  void payloadPCLPointCloud(const ::ExampleIdlData::BytesArray& _val_) { this->payloadPCLPointCloud_ = _val_; }
  void payloadPCLPointCloud(::ExampleIdlData::BytesArray&& _val_) { this->payloadPCLPointCloud_ = _val_; }

  bool operator==(const Msg& _other) const
  {
    return id_ == _other.id_ &&
      message_ == _other.message_ &&
      payloadEigen_ == _other.payloadEigen_ &&
      payloadOpenCVImage_ == _other.payloadOpenCVImage_ &&
      payloadPCLPointCloud_ == _other.payloadPCLPointCloud_;
  }

  bool operator!=(const Msg& _other) const
  {
    return !(*this == _other);
  }

};

}

#include "dds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/TopicTraits.hpp"
#include "org/eclipse/cyclonedds/topic/datatopic.hpp"

namespace org {
namespace eclipse {
namespace cyclonedds {
namespace topic {

template <> constexpr const char* TopicTraits<::ExampleIdlData::Msg>::getTypeName()
{
  return "ExampleIdlData::Msg";
}

template <> inline ddsi_sertype* TopicTraits<::ExampleIdlData::Msg>::getSerType()
{
  return static_cast<ddsi_sertype*>(new ddscxx_sertype<ExampleIdlData::Msg>());
}

template <> constexpr bool TopicTraits<::ExampleIdlData::Msg>::isSelfContained()
{
  return false;
}

}
}
}
}

namespace dds {
namespace topic {

template <>
struct topic_type_name<::ExampleIdlData::Msg>
{
    static std::string value()
    {
      return org::eclipse::cyclonedds::topic::TopicTraits<::ExampleIdlData::Msg>::getTypeName();
    }
};

}
}

REGISTER_TOPIC_TYPE(::ExampleIdlData::Msg)

namespace org{
namespace eclipse{
namespace cyclonedds{
namespace core{
namespace cdr{

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write__ExampleIdlData_BytesArray(T& streamer, const ::ExampleIdlData::BytesArray& instance) {
  (void)instance;
      if (!streamer.start_consecutive(false, true))
        return false;
      {
      uint32_t se_1 = uint32_t(instance.size());
      if (!write(streamer, se_1))
        return false;
      if (se_1 > 0 &&
          !write(streamer, instance[0], se_1))
        return false;
      }  //end sequence 1
      if (!streamer.finish_consecutive())
        return false;
  return true;
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read__ExampleIdlData_BytesArray(T& streamer, ::ExampleIdlData::BytesArray& instance) {
  (void)instance;
      if (!streamer.start_consecutive(false, true))
        return false;
      {
      uint32_t se_1 = uint32_t(instance.size());
      if (!read(streamer, se_1))
        return false;
      instance.resize(se_1);
      if (se_1 > 0 &&
          !read(streamer, instance[0], se_1))
        return false;
      }  //end sequence 1
      if (!streamer.finish_consecutive())
        return false;
  return true;
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move__ExampleIdlData_BytesArray(T& streamer, const ::ExampleIdlData::BytesArray& instance) {
  (void)instance;
      if (!streamer.start_consecutive(false, true))
        return false;
      {
      uint32_t se_1 = uint32_t(instance.size());
      if (!move(streamer, se_1))
        return false;
      if (se_1 > 0 &&
          !move(streamer, uint8_t(), se_1))
        return false;
      }  //end sequence 1
      if (!streamer.finish_consecutive())
        return false;
  return true;
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max__ExampleIdlData_BytesArray(T& streamer, const ::ExampleIdlData::BytesArray& instance) {
  (void)instance;
      if (!streamer.start_consecutive(false, true))
        return false;
      {
      uint32_t se_1 = 0;
      if (!max(streamer, se_1))
        return false;
      if (se_1 > 0 &&
          !max(streamer, uint8_t(), se_1))
        return false;
      }  //end sequence 1
      if (!streamer.finish_consecutive())
        return false;
      streamer.position(SIZE_MAX);
  return true;
}

template<>
entity_properties_t get_type_props<::ExampleIdlData::Msg>();

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool write(T& streamer, const ::ExampleIdlData::Msg& instance, entity_properties_t &props) {
  (void)instance;
  if (!streamer.start_struct(props))
    return false;
  bool firstcall = true;
  while (auto &prop = streamer.next_entity(props, firstcall)) {
    switch (prop.m_id) {
      case 0:
      if (!streamer.start_member(prop))
        return false;
      if (!write(streamer, instance.id()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(prop))
        return false;
      if (!write_string(streamer, instance.message(), 0))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(prop))
        return false;
      if (!write__ExampleIdlData_BytesArray(streamer, instance.payloadEigen()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(prop))
        return false;
      if (!write__ExampleIdlData_BytesArray(streamer, instance.payloadOpenCVImage()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(prop))
        return false;
      if (!write__ExampleIdlData_BytesArray(streamer, instance.payloadPCLPointCloud()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
    }
  }
  return streamer.finish_struct(props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool write(S& str, const ::ExampleIdlData::Msg& instance, bool as_key) {
  auto props = get_type_props<::ExampleIdlData::Msg>();
  str.set_mode(cdr_stream::stream_mode::write, as_key);
  return write(str, instance, props); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool read(T& streamer, ::ExampleIdlData::Msg& instance, entity_properties_t &props) {
  (void)instance;
  if (!streamer.start_struct(props))
    return false;
  bool firstcall = true;
  while (auto &prop = streamer.next_entity(props, firstcall)) {
    if (prop.ignore) {
      streamer.skip_entity(prop);
      continue;
    }
    switch (prop.m_id) {
      case 0:
      if (!streamer.start_member(prop))
        return false;
      if (!read(streamer, instance.id()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(prop))
        return false;
      if (!read_string(streamer, instance.message(), 0))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(prop))
        return false;
      if (!read__ExampleIdlData_BytesArray(streamer, instance.payloadEigen()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(prop))
        return false;
      if (!read__ExampleIdlData_BytesArray(streamer, instance.payloadOpenCVImage()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(prop))
        return false;
      if (!read__ExampleIdlData_BytesArray(streamer, instance.payloadPCLPointCloud()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      default:
      if (prop.must_understand_remote
       && streamer.status(must_understand_fail))
        return false;
      else
        streamer.skip_entity(prop);
      break;
    }
  }
  return streamer.finish_struct(props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool read(S& str, ::ExampleIdlData::Msg& instance, bool as_key) {
  auto props = get_type_props<::ExampleIdlData::Msg>();
  str.set_mode(cdr_stream::stream_mode::read, as_key);
  return read(str, instance, props); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool move(T& streamer, const ::ExampleIdlData::Msg& instance, entity_properties_t &props) {
  (void)instance;
  if (!streamer.start_struct(props))
    return false;
  bool firstcall = true;
  while (auto &prop = streamer.next_entity(props, firstcall)) {
    switch (prop.m_id) {
      case 0:
      if (!streamer.start_member(prop))
        return false;
      if (!move(streamer, instance.id()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(prop))
        return false;
      if (!move_string(streamer, instance.message(), 0))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(prop))
        return false;
      if (!move__ExampleIdlData_BytesArray(streamer, instance.payloadEigen()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(prop))
        return false;
      if (!move__ExampleIdlData_BytesArray(streamer, instance.payloadOpenCVImage()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(prop))
        return false;
      if (!move__ExampleIdlData_BytesArray(streamer, instance.payloadPCLPointCloud()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
    }
  }
  return streamer.finish_struct(props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool move(S& str, const ::ExampleIdlData::Msg& instance, bool as_key) {
  auto props = get_type_props<::ExampleIdlData::Msg>();
  str.set_mode(cdr_stream::stream_mode::move, as_key);
  return move(str, instance, props); 
}

template<typename T, std::enable_if_t<std::is_base_of<cdr_stream, T>::value, bool> = true >
bool max(T& streamer, const ::ExampleIdlData::Msg& instance, entity_properties_t &props) {
  (void)instance;
  if (!streamer.start_struct(props))
    return false;
  bool firstcall = true;
  while (auto &prop = streamer.next_entity(props, firstcall)) {
    switch (prop.m_id) {
      case 0:
      if (!streamer.start_member(prop))
        return false;
      if (!max(streamer, instance.id()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 1:
      if (!streamer.start_member(prop))
        return false;
      if (!max_string(streamer, instance.message(), 0))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 2:
      if (!streamer.start_member(prop))
        return false;
      if (!max__ExampleIdlData_BytesArray(streamer, instance.payloadEigen()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 3:
      if (!streamer.start_member(prop))
        return false;
      if (!max__ExampleIdlData_BytesArray(streamer, instance.payloadOpenCVImage()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
      case 4:
      if (!streamer.start_member(prop))
        return false;
      if (!max__ExampleIdlData_BytesArray(streamer, instance.payloadPCLPointCloud()))
        return false;
      if (!streamer.finish_member(prop))
        return false;
      break;
    }
  }
  return streamer.finish_struct(props);
}

template<typename S, std::enable_if_t<std::is_base_of<cdr_stream, S>::value, bool> = true >
bool max(S& str, const ::ExampleIdlData::Msg& instance, bool as_key) {
  auto props = get_type_props<::ExampleIdlData::Msg>();
  str.set_mode(cdr_stream::stream_mode::max, as_key);
  return max(str, instance, props); 
}

} //namespace cdr
} //namespace core
} //namespace cyclonedds
} //namespace eclipse
} //namespace org

#endif // DDSCXX_EXAMPLEIDLDATA_HPP
