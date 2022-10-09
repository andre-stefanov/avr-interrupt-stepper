cmake_minimum_required(VERSION 3.23)
project(
        avr-interrupt-stepper
        VERSION 1.0
        DESCRIPTION "Interrupt driven stepper motor library based on AVR466"
        LANGUAGES CXX C ASM
)

set(CMAKE_CXX_STANDARD 17)

#[[include(cmake/generic-gcc-avr.cmake)]]

include_directories(include)

add_subdirectory(test)