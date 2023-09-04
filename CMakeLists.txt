cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(sd-2d-lidar-scanner)

message("Library configuration:")
if(DEFINED CMAKE_TOOLCHAIN_FILE)
	message(STATUS "Using ${CMAKE_TOOLCHAIN_FILE} as toolchain file (possibly build from source).")
else()
	message(STATUS "Attempting to use installed libraries.")
endif()

add_subdirectory(src)
