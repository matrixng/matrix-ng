project(malos_service C CXX)
cmake_minimum_required(VERSION 2.8)

add_definitions(-std=c++11)

FIND_LIBRARY(HAL_LIB NAMES matrix_creator_hal)
message(STATUS "HAL found => " "${HAL_LIB}")

FIND_LIBRARY(GFLAGS_LIB NAMES gflags)
message(STATUS "gflags found =>" "${GFLAGS_LIB}") 

find_package(Threads)

add_executable(firmware_info firmware_info.cpp)
  set_property(TARGET firmware_info PROPERTY CXX_STANDARD 11)
  target_link_libraries(firmware_info ${HAL_LIB})
  target_link_libraries(firmware_info ${CMAKE_THREAD_LIBS_INIT})

add_executable(fpga_info fpga_info.cpp)
  set_property(TARGET fpga_info PROPERTY CXX_STANDARD 11)
  target_link_libraries(fpga_info ${HAL_LIB})
  target_link_libraries(fpga_info ${CMAKE_THREAD_LIBS_INIT})