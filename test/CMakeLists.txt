cmake_minimum_required(VERSION 3.10)
project(drone_test CXX)
set(CMAKE_CXX_STANDARD 14)

include_directories(
    ${CMAKE_SOURCE_DIR}/stubs       # Arduino.h stub (searched first)
    ${CMAKE_SOURCE_DIR}/../main/src # project headers
)

add_compile_definitions(DEBUG_MODE=0)

add_executable(test_runner
    test_runner.cpp
    ../main/control/AutoController.cpp
    ../main/control/ManualController.cpp
    ../main/control/ModeManager.cpp
    ../main/navigation/Navigator.cpp
)
