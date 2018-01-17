cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 17)

# set (MUJOCO_DIR "/home/tao/.mujoco/mjpro150")
set (MUJOCO_ROOT_DIR "/home/tao/.mujoco/mjpro150")
set (Ipopt_ROOT_DIR "/usr/local")
set (Range3_INCLUDE_DIR "/home/tao/src/range-v3/include")
include(../cmake/FindMujoco.cmake)
include(../cmake/FindIpopt.cmake)

if (NOT MUJOCO_FOUND)
	message(FATAL_ERROR "Failed to find mujoco at " ${MUJOCO_ROOT_DIR})
endif()

add_library(trajectoryOptimizationLib INTERFACE)
target_include_directories(trajectoryOptimizationLib INTERFACE
	${Range3_INCLUDE_DIR}
	"${PROJECT_SOURCE_DIR}/trajectoryOptimizationLibrary")

enable_testing()
find_package(GTest REQUIRED)
add_executable(costTest test/costTest.cpp)
target_link_libraries(costTest PUBLIC GTest::GTest GTest::Main)
target_link_libraries(costTest PUBLIC trajectoryOptimizationLib)

add_executable(constraintTest test/constraintTest.cpp)
target_link_libraries(constraintTest PUBLIC GTest::GTest GTest::Main)
target_link_libraries(constraintTest PUBLIC trajectoryOptimizationLib)

add_executable(dynamicTest test/dynamicTest.cpp)
target_link_libraries(dynamicTest PUBLIC GTest::GTest GTest::Main)
target_link_libraries(dynamicTest PUBLIC trajectoryOptimizationLib)

add_test(costTest costTest)
add_test(constriantTest constraintTest)
add_test(dynamicTest dynamicTest)



