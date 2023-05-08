cmake_minimum_required(VERSION 3.10)
project(hmm_traj_planning)
set(CMAKE_CXX_STANDARD 17)

find_package(GTest REQUIRED)
find_package(drake CONFIG REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories("${PROJECT_SOURCE_DIR}/include")

add_library(robotLibs
  src/utility.cpp
  src/robot.cpp
)

target_link_libraries(robotLibs
  Eigen3::Eigen
  drake::drake
)

add_executable(test_robot src/test_robot.cpp)
target_link_libraries(test_robot
  robotLibs
)

add_library(myLibs
  src/problem.cpp
  src/rrt_cspace.cpp
)
target_link_libraries(myLibs
  robotLibs
)
#
add_executable(test_rrt src/test_rrt.cpp)
target_link_libraries(test_rrt
  myLibs
)

# include(FetchContent)
# FetchContent_Declare(
#   googletest
#   URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
# )
#
# enable_testing()
#
# add_executable(test_utility test/test_utility.cpp)
# target_link_libraries(test_utility
#   GTest::gtest_main
# )
#
# add_executable(test_robot test/test_robot.cpp)
# target_link_libraries(test_robot
#   robotLibs
#   GTest::gtest_main
# )
#
# add_executable(test_all 
#   test/main.cpp 
#   test/test_robot.cpp
#   test/test_utility.cpp
# )
#
# target_link_libraries(test_all
#   robotLibs
#   GTest::gtest_main
# )
#
# set_target_properties(
#   test_utility
#   test_robot 
#   test_all
#   PROPERTIES
#   RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/test
# )
#
# include(GoogleTest)
# gtest_discover_tests(test_utility)
# gtest_discover_tests(test_robot)
file(COPY data DESTINATION ${CMAKE_BINARY_DIR})
