cmake_minimum_required(VERSION 3.11)

project(triangles)

set(SOURCES
        double_compare.cpp 
        point.cpp 
        vector.cpp 
        plane.cpp 
        segment.cpp
        triangle.cpp 
        octree.cpp 
        main.cpp
)

add_executable(triangles ${SOURCES})

set_target_properties(
        triangles PROPERTIES 
        CXX_STANDART 17
        CXX_STANDARD_REQUIRED ON
)