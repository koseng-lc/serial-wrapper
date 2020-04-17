# Serial Wrapper
Simple serial communcation interface between Arduino and PC, this actually wrapper for Boost.ASIO.
This project is part of my capstone project that is called MICRON.

## Features
* Support two-way communication

## Example
### PC side
```cpp
#include <iostream>
#include <csignal>

#include <serial_wrapper/serial_wrapper.hpp>

micron::SerialWrapper<> sw("/dev/ttyUSB0", micron::baud_rate_t(115200));

void sigHandler(int sig){
    (void)sig;
    sw.stop();
}

int main(int argc, char** argv){
    std::string data( "Test from PC" );
    std::string received_data;
    while(sw.isRunning()){
        
        sw.sendData(data);

        received_data = sw.getData();
        std::cout << "Received data : " << received_data << std::endl;

        boost::this_thread::sleep_for(boost::chrono::milliseconds{100});
    }

    return 0;
}
```
### Arduino side
```cpp
#include <ArduinoJson.h>
#include "serial_wrapper_arduino.hpp"
#include "circular_buffer.hpp"

micron::SerialWrapper* sw;

void setup() {
  Serial.begin(115200);
  sw = new micron::SerialWrapper(&Serial);
}

void loop() {      
  String received_data = sw->getData();
  //-- do something here with the data
  
  String data = "Test from Arduino";
  sw->sendData(data);      
  delay(100);
}

void serialEvent(){
  sw->receiveRoutine();
}
```
## Installation
There are no need installation here just setup your `CMakeLists.txt` project
```cmake
...
add_subdirectory(serial-wrapper) # this is the folder where you place serial-wrapper
target_link_libraries(<YOUR_TARGET> ... serial_wrapper ...)
...
```
### Dependencies
* Boost
