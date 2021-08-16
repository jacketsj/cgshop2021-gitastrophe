cmake_minimum_required(VERSION 3.5)

project(cgshop2021)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)

set(SRC ${CMAKE_SOURCE_DIR}/src)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
include_directories("${SRC}")
set(CMAKE_CXX_FLAGS "-Wall -pthread -g -O3 -funroll-loops")

set(JSON_BuildTests OFF CACHE INTERNAL "")
add_subdirectory(json)

#find_package(Threads)
find_package(X11)

#add_library(sl SHARED IMPORTED)

add_executable(vis ${SRC}/vis/vis.cpp)
add_executable(gif_vis ${SRC}/vis/gif_vis.cpp)
add_executable(optimize ${SRC}/optimize.cpp)

target_link_libraries(optimize PRIVATE nlohmann_json::nlohmann_json stdc++fs)

target_link_libraries(vis X11)
target_link_libraries(gif_vis X11)
