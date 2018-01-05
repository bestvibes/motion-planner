cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 17)

# set (MUJOCO_DIR "/home/tao/.mujoco/mjpro150")
set (MUJOCO_ROOT_DIR "/home/tao/.mujoco/mjpro150")
set (Ipopt_ROOT_DIR "/usr/local")
set (Range3_INCLUDE_DIR "/home/tao/src/range-v3/include")
include(FindMujoco.cmake)
include(FindIpopt.cmake)

if (NOT MUJOCO_FOUND)
	message(FATAL_ERROR "Failed to find mujoco at " ${MUJOCO_ROOT_DIR})
endif()

find_package(Eigen3 REQUIRED)
message(STATUS ${EIGEN3_INCLUDE_DIR})
# message(STATUS "eigen path is : $ENV{EIGEN3_INCLUDE_DIR}")
# INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )


add_executable(simulate basic.cpp)
target_include_directories(simulate PUBLIC ${MUJOCO_INCLUDE_DIR})
target_link_libraries(simulate ${MUJOCO_LIB} ${MUJOCO_EXTERN_LIBS})

add_executable(derivative derivative.cpp)
target_include_directories(derivative PUBLIC ${MUJOCO_INCLUDE_DIR})
target_link_libraries(derivative ${MUJOCO_LIB} ${MUJOCO_EXTERN_LIBS})


add_executable(traj_opt traj_opt_main.cpp)
target_include_directories(traj_opt PUBLIC ${Ipopt_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR})
target_link_libraries(traj_opt PUBLIC ${Ipopt_LIB} ${Ipopt_THIRDPARTY_LIBS})
#
add_executable(test_eigen test_eigen.cpp)
target_include_directories(test_eigen PUBLIC ${Ipopt_INCLUDE_DIR}
	${EIGEN3_INCLUDE_DIR} ${Range3_INCLUDE_DIR})
target_link_libraries(test_eigen PUBLIC ${Ipopt_LIB} ${Ipopt_THIRDPARTY_LIBS})

