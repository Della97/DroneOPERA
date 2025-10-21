cmake_minimum_required(VERSION 3.10)
project(simulation_sender)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)
add_executable(simulation_sender main.cpp)
find_package(Boost REQUIRED system thread)
target_link_libraries(simulation_sender Boost::system pthread)