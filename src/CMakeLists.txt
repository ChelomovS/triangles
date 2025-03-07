cmake_minimum_required(VERSION 3.11)

project(triangles)

set(SOURCES
        main.cpp
)

add_executable(triangles ${SOURCES})

set_target_properties(
        triangles PROPERTIES 
        CXX_STANDART 23
        CXX_STANDARD_REQUIRED ON
)