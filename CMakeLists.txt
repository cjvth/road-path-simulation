cmake_minimum_required(VERSION 3.28)
project(paths)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Osmium REQUIRED COMPONENTS xml)
include_directories(SYSTEM ${OSMIUM_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 20)

add_subdirectory(cpp)
