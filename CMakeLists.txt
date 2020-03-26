cmake_minimum_required(VERSION 2.8.3)
project(serial_wrapper)

find_package(Boost REQUIRED COMPONENTS system thread)

add_library(serial_wrapper serial_wrapper.hpp)
set_target_properties(serial_wrapper PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(serial_wrapper Boost::system Boost::thread)
