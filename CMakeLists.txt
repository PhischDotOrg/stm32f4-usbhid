#-
# $Copyright$
#
cmake_minimum_required(VERSION 3.12)
project(STM32F4-UsbHid)

###############################################################################
# Enable C++17 Support according to:
#
# https://crascit.com/2015/03/28/enabling-cxx11-in-cmake/
#
# Google Test requires > C++11.
###############################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###############################################################################
# Include Unit Testing / Code Coverage Support
###############################################################################
include(common/UnitTest.cmake)
include(common/Coverage.cmake)

###############################################################################
# Project-specific configuration
###############################################################################
set(PROJECT_NAME    ${CMAKE_PROJECT_NAME}) # FIXME This is redundant
set(TARGET_NAME     firmware)
set(TARGET_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/UsbDescriptors.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/common/stm32f4/usb/UsbIrqViaSTM32F4.cpp
)

set(STM32F4_BOARD   STM32F4_Nucleo_F411RE   CACHE STRING "STM32F4 Board Type")

# add_definitions("-DUSB_DEBUG")

###############################################################################
# Include a common CMake File that contains the stuff which would normally go
# in a top-level CMake File.
###############################################################################
add_subdirectory(common)
