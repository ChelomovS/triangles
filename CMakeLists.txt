cmake_minimum_required(VERSION 3.11)

project(triangles)

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

aux_source_directory(src SRC_FILES)

add_executable(triangles ${SRC_FILES})

target_include_directories(triangles PRIVATE ${INCLUDE_DIR})

set(CMAKE_CXX_FLAGS_DEBUG "-Wall -Wextra -Wpedantic -g -O0 -DDEBUG \
                           -Wmissing-declarations -Wcast-align \
                           -Wunused -Wshadow -Wsign-conversion \
                           -fsanitize=address -fsanitize=undefined")

set_target_properties(
    triangles PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

add_subdirectory(tests)