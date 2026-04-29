cmake_minimum_required(VERSION 3.10)
project(drone_test CXX)
set(CMAKE_CXX_STANDARD 14)

include_directories(
    ${CMAKE_SOURCE_DIR}/stubs       # Arduino.h stub (searched first)
    ${CMAKE_SOURCE_DIR}/../main     # project headers
)

add_executable(test_runner
    test_runner.cpp
    ../main/control/ManualController.cpp
    ../main/control/ModeManager.cpp
)
