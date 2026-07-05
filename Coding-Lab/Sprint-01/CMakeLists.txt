cmake_minimum_required(VERSION 3.10)

project(transforms)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

include_directories(include)

add_executable(transforms
    src/transform2D.cpp
    src/transform3D.cpp
    src/main.cpp
)

target_link_libraries(transforms stdc++fs)