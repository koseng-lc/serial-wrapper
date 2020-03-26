# Serial Wrapper
Simple serial communcation interface between Arduino and PC, this actually wrapper for Boost.ASIO.
The features isn't complete yet, at this point Arduino only possible to transmit and PC vice versa, i will complement the features in future if possible.
This project is part of my capstone project that is called MICRON.
## Example
### PC side
```cpp
#include <iostream>
#include <csignal>

#include <serial_wrapper/serial_wrapper.hpp>

micron::SerialWrapper<> sw("/dev/ttyUSB0", micron::baud_rate_t(9600));

void sigHandler(int sig){
    (void)sig;
    sw.stop();
}

int main(int argc, char** argv){
    constexpr std::size_t BULK_SIZE(10);
    while(sw.isRunning()){
        std::vector<double> bulk_data;
        sw.getBulkData(&bulk_data, BULK_SIZE);

        std::cout << "Received data : ";
        for(const auto& d:bulk_data)
            std::cout << d << "  ";
        std::cout << std::endl;

        boost::this_thread::sleep_for(boost::chrono::milliseconds{100});
    }

    return 0;
}
```
### Arduino side
```cpp
#include "serial_wrapper_arduino.hpp"

micron::SerialWrapper* sw;

void setup() {
  Serial.begin(9600);
  sw = new micron::SerialWrapper(&Serial);
}
static const size_t BULK_SIZE(10);
void loop() {
  double bulk_data[BULK_SIZE];
  for(int i(0); i < BULK_SIZE; i++){
    bulk_data[i] = (rand()%100)* 0.1;
  }
  sw->sendBulkData(bulk_data, BULK_SIZE);  
  delay(100);
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
