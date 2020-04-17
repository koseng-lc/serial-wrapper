/**
*   @author : koseng(Lintang)
*   @brief : Simple serial wrapper for Boost.ASIO, this code is part of my capstone project
*/
#pragma once

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/circular_buffer.hpp>

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
    inline auto isRunning() const -> bool{return is_running_;}    

    auto getData() -> std::string;
    auto sendData(std::string _data) -> void;

private:
    inline auto receive() -> void;
    auto receiveCompletion(const boost::system::error_code& _e, std::size_t _received_data);
    auto transmit() -> void;
    auto transmitCompletion(const boost::system::error_code& _e);

private:
    static constexpr std::size_t READ_BUFFER_SIZE{50};
    uchar_t buffer_[READ_BUFFER_SIZE];
    boost::circular_buffer<std::string> rpacket_buffer_;
    boost::circular_buffer<std::string> tpacket_buffer_;
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
    : rpacket_buffer_(10)
    , tpacket_buffer_(10)
    , is_running_(true)
    , port_(io_){
    port_.open(_dev_name);
    port_.set_option(_baud_rate);
    port_.set_option(_stop_bits);
    port_.set_option(_parity);
    port_.set_option(_char_size);
    port_.set_option(_flow_control);
    io_.post(boost::bind(&SerialWrapper::receive, this));
    receive_thread_ = boost::thread{boost::bind(&boost::asio::io_service::run, &io_)};
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
    : rpacket_buffer_(10)
    , tpacket_buffer_(10)
    , is_running_(true)
    , port_(io_){
    port_.open(_dev_name);
    port_.set_option(_baud_rate);
    port_.set_option(micron::stop_bits_t(micron::stop_bits_t::one));
    port_.set_option(micron::parity_t(micron::parity_t::none));
    port_.set_option(micron::char_size_t(8));
    port_.set_option(micron::flow_control_t(micron::flow_control_t::none));    
    io_.post(boost::bind(&SerialWrapper::receive, this));
    receive_thread_ = boost::thread{boost::bind(&boost::asio::io_service::run, &io_)};
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
                   FlowControl>::receive() -> void{
    MICRON_ASIO_EXCEPTIONAL_HANDLER(
        boost::asio::async_read(port_, boost::asio::buffer(buffer_, READ_BUFFER_SIZE),
                                boost::bind(&SerialWrapper::receiveCompletion,
                                            this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));
    )
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
        FlowControl>::receiveCompletion(const boost::system::error_code& _e, std::size_t _len){
    if(_e){
        //-- do something here, to handle the error
    }else{
        static std::string packet;
        for(int i(0);i < READ_BUFFER_SIZE; i++){
            if(buffer_[i] == '\n'){
                rpacket_buffer_.push_back(packet);
                packet.clear();
            }else{
                packet += buffer_[i];
            }
        }
//        for(const auto& p:rpacket_buffer_)
//            std::cout << p << std::endl;
        receive();
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
                   FlowControl>::transmit() -> void{
    MICRON_ASIO_EXCEPTIONAL_HANDLER(
        boost::asio::async_write(port_, boost::asio::buffer(tpacket_buffer_.front(),tpacket_buffer_.front().size()),
                                 boost::bind(&SerialWrapper::transmitCompletion,
                                             this,
                                             boost::asio::placeholders::error));
        tpacket_buffer_.pop_front();
    )
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
        FlowControl>::transmitCompletion(const boost::system::error_code& _e){
    if(_e){
        //-- do something here, to handle the error
    }else{
        if(tpacket_buffer_.size()){
            transmit();
        }
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
                   FlowControl>::getData() -> std::string{
    if(rpacket_buffer_.empty())
        return std::string("");
    std::string temp( rpacket_buffer_.front() );
    rpacket_buffer_.pop_front();
    return temp;
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
                   FlowControl>::sendData(std::string _data) -> void{
    tpacket_buffer_.push_back(_data+"\n");
    io_.post(boost::bind(&SerialWrapper::transmit, this));
}

} // namespace micron
