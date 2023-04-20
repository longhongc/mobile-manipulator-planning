cmake_minimum_required(VERSION 3.10)
project(hw3_RRT_with_dynamic)
set(CMAKE_CXX_STANDARD 17)

find_package(GTest REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_library(robotLibs
  src/robot.cpp
)

# add_executable(solution src/main.cpp)
# target_link_libraries(solution
#   myLibs
# )

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)

enable_testing()

add_executable(test_utility test/test_utility.cpp)
target_link_libraries(test_utility
  GTest::gtest_main
)

add_executable(test_robot test/test_robot.cpp)
target_link_libraries(test_robot
  robotLibs
  GTest::gtest_main
)

set_target_properties(
  test_utility
  test_robot 
  PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test
)

include(GoogleTest)
gtest_discover_tests(test_utility)
gtest_discover_tests(test_robot)
