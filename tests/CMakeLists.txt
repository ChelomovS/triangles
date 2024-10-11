cmake_minimum_required(VERSION 3.11)

project(gtest)
cmake_minimum_required(VERSION 3.11)

if(NOT DEFINED INCLUDE_DIR)
    message(WARNING "INCLUDE_DIR is not defined.")
endif()

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES_FOR_TESTING
    ../src/point.cpp
    ../src/vector.cpp
    ../src/plane.cpp
    ../src/segment.cpp
    ../src/triangle.cpp
    ../src/double_compare.cpp
)

aux_source_directory(unit UNIT_TESTS)

enable_testing()

add_subdirectory(googletest)

add_executable(unit_tests ${SOURCES_FOR_TESTING} ${UNIT_TESTS})

target_include_directories(unit_tests 
                           PRIVATE
                           ${CMAKE_CURRENT_SOURCE_DIR}/googletest/include 
                           ${INCLUDE_DIR}
)

target_link_libraries(unit_tests gtest_main gmock_main)
