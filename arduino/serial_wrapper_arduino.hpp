#pragma once

#include <Arduino.h>

namespace micron{

class SerialWrapper{
public:
  SerialWrapper(Stream* _stream);
  ~SerialWrapper();

  inline void transmit(unsigned char _data);
  template <typename Type>
  void transmitPacket(Type _data);
  template <typename Type>
  void sendBulkData(Type* _bulk_data, size_t _bulk_size);
  
private:
  Stream* stream_;
};

SerialWrapper::SerialWrapper(Stream* _stream){
  stream_ = _stream;
}

SerialWrapper::~SerialWrapper(){
  stream_ = NULL;
}

void SerialWrapper::transmit(unsigned char _data){
  stream_->write(_data);
}

template <typename Type>
void SerialWrapper::transmitPacket(Type _data){
  String str_data(_data);
  for(size_t i(0); i < str_data.length(); i++){
    transmit( str_data.charAt(i) );
  }
  transmit('\n');
}

template <typename Type>
void SerialWrapper::sendBulkData(Type* _bulk_data, size_t _bulk_size){
  for(size_t i(0); i < _bulk_size; i++){
    transmitPacket(_bulk_data[i]);
  }
}

}
