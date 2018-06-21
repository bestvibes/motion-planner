cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 17)

#Make sure that custom modules like FindGoogleTest are found
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

find_package(GoogleTest REQUIRED MODULE)

add_executable(costTest src/costTest.cpp)
target_link_libraries(costTest PUBLIC gtest_main)
target_link_libraries(costTest PUBLIC TrajectoryOptimization::TrajectoryOptimizationLib)

add_executable(constraintTest src/constraintTest.cpp)
target_link_libraries(constraintTest PUBLIC gtest_main)
target_link_libraries(constraintTest PUBLIC TrajectoryOptimization::TrajectoryOptimizationLib)

add_executable(dynamicTest src/dynamicTest.cpp)
target_link_libraries(dynamicTest PUBLIC gtest_main)
target_link_libraries(dynamicTest PUBLIC TrajectoryOptimization::TrajectoryOptimizationLib)

add_executable(utilitiesTest src/utilitiesTest.cpp)
target_link_libraries(utilitiesTest PUBLIC gtest_main)
target_link_libraries(utilitiesTest PUBLIC TrajectoryOptimization::TrajectoryOptimizationLib)

add_executable(optimizerTest src/optimizerTest.cpp)
target_link_libraries(optimizerTest PUBLIC gtest_main)
target_link_libraries(optimizerTest PUBLIC TrajectoryOptimization::TrajectoryOptimizationLib)

add_executable(derivativeTest src/derivativeTest.cpp)
target_link_libraries(derivativeTest PUBLIC gtest_main)
target_link_libraries(derivativeTest PUBLIC TrajectoryOptimization::TrajectoryOptimizationLib)

add_test(costTest costTest)
add_test(constriantTest constraintTest)
add_test(dynamicTest dynamicTest)
add_test(utilitiesTest utilitiesTest)
add_test(optimizerTest optimizerTest)
add_test(derivativeTest derivativeTest)
