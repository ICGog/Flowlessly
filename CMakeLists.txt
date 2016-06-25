project(Flowlessly)

cmake_minimum_required(VERSION 2.8)

enable_testing()

set(Flowlessly_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_MODULE_PATH ${Flowlessly_SOURCE_DIR}/cmake)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${Flowlessly_BINARY_DIR})

# CMake libraries
include(CheckLibraryExists)
include(CheckFunctionExists)
include(ExternalProject)
include(CheckCXXSourceCompiles)
include(CheckCXXCompilerFlag)

# Build options
include(BuildOptions)

# Compiler config
include(CXXCompilerConfig)

# External dependencies
include(ExternalDependencies)

# Headers use paths relative to src/
include_directories("${CMAKE_SOURCE_DIR}/src/")

# Set subdirectories
add_subdirectory(src)
add_subdirectory(src/graphs)
add_subdirectory(src/misc)
add_subdirectory(src/solvers)
