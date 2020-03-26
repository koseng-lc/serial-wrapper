/**
*   @author : koseng(Lintang)
*   @brief : Simple serial wrapper for Boost.ASIO, this code is part of my capstone project
*/
#pragma once

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>

#define MICRON_ASIO_EXCEPTIONAL_HANDLER(x) try{x}catch(boost::system::system_error &e){std::cerr << e.what() << std::endl;}

namespace micron{

using baud_rate_t = boost::asio::serial_port_base::baud_rate;
using stop_bits_t = boost::asio::serial_port_base::stop_bits;
using parity_t = boost::asio::serial_port_base::parity;
using char_size_t = boost::asio::serial_port_base::character_size;
using flow_control_t = boost::asio::serial_port::flow_control;

template <typename BaudRate = baud_rate_t,
          typename StopBits = stop_bits_t,
          typename Parity = parity_t,
          typename CharSize = char_size_t,
          typename FlowControl = flow_control_t>
class SerialWrapper{
public:
    using uchar_t = unsigned char;

    SerialWrapper(std::string _dev_name,
                  BaudRate _baud_rate,
                  StopBits _stop_bits,
                  Parity _parity,
                  CharSize _char_size,
                  FlowControl _flow_control);
    SerialWrapper(std::string _dev_name, BaudRate _baud_rate);
    ~SerialWrapper(){ stop(); }

    inline auto stop() -> void{is_running_ = false;}
    inline auto isRunning() -> bool{return is_running_;}
    template <typename Type>
    auto getBulkData(std::vector<Type>* _bulk_data, std::size_t _bulk_size) -> void;

private:
    inline auto receive(uchar_t* _data) -> void;
    template <typename Type>
    auto receivePacket(Type* _data) -> void;
    auto receiveRoutine() -> void;

    template<typename T>
    inline auto str2num(std::string _str, T* t) -> bool{
        try{
            *t = boost::lexical_cast<T>(_str);
        }catch(boost::bad_lexical_cast& e){
//            std::cerr << e.what() << std::endl;
            return false;
        }
        return true;
    }

private:
    std::atomic<bool> is_running_;

    boost::thread receive_thread_;

    boost::asio::io_service io_;
    boost::asio::serial_port port_;
};
template <typename BaudRate,
          typename StopBits,
          typename Parity,
          typename CharSize,
          typename FlowControl>
SerialWrapper<BaudRate,
              StopBits,
              Parity,
              CharSize,
              FlowControl>::SerialWrapper(std::string _dev_name,
                                          BaudRate _baud_rate,
                                          StopBits _stop_bits,
                                          Parity _parity,
                                          CharSize _char_size,
                                          FlowControl _flow_control)
    : is_running_(true)
    , port_(io_){
    port_.open(_dev_name);
    port_.set_option(_baud_rate);
    port_.set_option(_stop_bits);
    port_.set_option(_parity);
    port_.set_option(_char_size);
    port_.set_option(_flow_control);
//    receive_thread_ = boost::thread{boost::bind(&SerialWrapper::receiveRoutine, this)};
}

template <typename BaudRate,
          typename StopBits,
          typename Parity,
          typename CharSize,
          typename FlowControl>
SerialWrapper<BaudRate,
              StopBits,
              Parity,
              CharSize,
              FlowControl>::SerialWrapper(std::string _dev_name,
                                          BaudRate _baud_rate)
    : is_running_(true)
    , port_(io_){
    port_.open(_dev_name);
    port_.set_option(_baud_rate);
    port_.set_option(micron::stop_bits_t(micron::stop_bits_t::one));
    port_.set_option(micron::parity_t(micron::parity_t::none));
    port_.set_option(micron::char_size_t(8));
    port_.set_option(micron::flow_control_t(micron::flow_control_t::none));
//    receive_thread_ = boost::thread{boost::bind(&SerialWrapper::receiveRoutine, this)};
}

template <typename BaudRate,
          typename StopBits,
          typename Parity,
          typename CharSize,
          typename FlowControl>
auto SerialWrapper<BaudRate,
                   StopBits,
                   Parity,
                   CharSize,
                   FlowControl>::receive(uchar_t* _data) -> void{
    MICRON_ASIO_EXCEPTIONAL_HANDLER(
        boost::asio::read(port_, boost::asio::buffer(_data, 1));
    )
}

template <typename BaudRate,
          typename StopBits,
          typename Parity,
          typename CharSize,
          typename FlowControl>
template <typename Type>
auto SerialWrapper<BaudRate,
                   StopBits,
                   Parity,
                   CharSize,
                   FlowControl>::receivePacket(Type* _data) -> void{
    uchar_t d;
    while(1){
        receive(&d);
        if(d == '\n')
            break;
        *_data += d;
    }
}

template <typename BaudRate,
          typename StopBits,
          typename Parity,
          typename CharSize,
          typename FlowControl>
template <typename Type>
auto SerialWrapper<BaudRate,
                   StopBits,
                   Parity,
                   CharSize,
                   FlowControl>::getBulkData(std::vector<Type> *_bulk_data, std::size_t _bulk_size) -> void{
    assert(_bulk_size > 0 && "Bulk can't be zero.");
    Type t;
    for(std::size_t i(0); i < _bulk_size; i++){
        std::string d;
        receivePacket(&d);
        if(!str2num(d,&t))
            t = std::numeric_limits<Type>::quiet_NaN();
        _bulk_data->emplace_back(t);
    }
}

template <typename BaudRate,
          typename StopBits,
          typename Parity,
          typename CharSize,
          typename FlowControl>
auto SerialWrapper<BaudRate,
                   StopBits,
                   Parity,
                   CharSize,
                   FlowControl>::receiveRoutine() -> void{
    while(is_running_){

        boost::this_thread::sleep_for(boost::chrono::milliseconds{100});
    }
}

} // namespace micron
