cmake_minimum_required(VERSION 3.13 FATAL_ERROR)
set(CMAKE_C_STANDARD 99)

set(CORE_LIB_DIR ${CMAKE_SOURCE_DIR}/azure_rtos)

# Set the toolchain if not defined
set(CMAKE_TOOLCHAIN_FILE "cmake/arm-gcc-cortex-m4.cmake")

include(cmake/utilities.cmake)

# Define the Project
project(gd32_openfirmwaremower_main C ASM)

add_subdirectory(lib)
add_subdirectory(app)
