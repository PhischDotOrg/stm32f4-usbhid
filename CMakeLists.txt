#-
# $Copyright$
#
cmake_minimum_required(VERSION 3.12)
project(STM32F4-UsbHid)

###############################################################################
# Project-specific configuration
###############################################################################
set(PROJECT_NAME    ${CMAKE_PROJECT_NAME}) # FIXME This is redundant
set(TARGET_NAME     firmware)
set(TARGET_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

set(STM32F4_BOARD   STM32F4_Discovery   CACHE STRING "STM32F4 Board Type")

###############################################################################
# Include a common CMake File that contains the stuff which would normally go
# in a top-level CMake File.
###############################################################################
add_subdirectory(common)
