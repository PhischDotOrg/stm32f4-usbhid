#-
# $Copyright$
#
cmake_minimum_required(VERSION 3.12)

###############################################################################
# Project-specific configuration
###############################################################################
set(PROJECT_NAME    stm32f4-minimal)
set(TARGET_NAME     firmware)
set(TARGET_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

###############################################################################
# Include a common CMake File that contains the stuff which would normally go
# in a top-level CMake File.
###############################################################################
add_subdirectory(common)
