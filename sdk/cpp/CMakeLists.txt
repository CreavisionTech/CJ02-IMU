cmake_minimum_required(VERSION 3.10)
project(cj02_imu_example CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header-only SDK — just include the directory
add_executable(cj02_example src/example.cpp)
target_include_directories(cj02_example PRIVATE include)

# Threads
find_package(Threads REQUIRED)
target_link_libraries(cj02_example Threads::Threads)

if(WIN32)
    # Link Windows serial API
    target_link_libraries(cj02_example setupapi)
endif()
