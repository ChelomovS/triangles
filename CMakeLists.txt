cmake_minimum_required(VERSION 3.11)

project(triangles)

find_package(GTest REQUIRED)
enable_testing()

set(CMAKE_CXX_STANDARD          23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

aux_source_directory(src SRC_FILES)

add_executable(triangles ${SRC_FILES})

target_include_directories(triangles PRIVATE ${INCLUDE_DIR})

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -Wpedantic -g -O0 -DDEBUG \
                           -Wmissing-declarations -Wcast-align \
                           -Wunused -Wshadow -Wsign-conversion \
                           -fsanitize=address -fsanitize=undefined")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set_target_properties(
    triangles PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
)

add_subdirectory(tests)

target_link_libraries(triangles ${GTEST_BOTH_LIBRARIES})