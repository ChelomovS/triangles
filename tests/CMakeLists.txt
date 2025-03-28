cmake_minimum_required(VERSION 3.11)

project(unit_testing)
cmake_minimum_required(VERSION 3.11)

if(NOT DEFINED INCLUDE_DIR)
    message(WARNING "INCLUDE_DIR is not defined.")
endif()

find_package(GTest REQUIRED)

set(CMAKE_CXX_STANDARD          23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

aux_source_directory(unit UNIT_TESTS)

add_executable(unit_tests ${UNIT_TESTS})

target_include_directories(unit_tests PRIVATE ${INCLUDE_DIR} ${GTEST_INCLUDE_DIRS})

enable_testing()

target_link_libraries(unit_tests ${GTEST_BOTH_LIBRARIES})

add_test(NAME unit_tests COMMAND unit_tests)