cmake_minimum_required(VERSION 3.2)
set(CMAKE_CXX_STANDARD 11)

# set (MUJOCO_DIR "/home/tao/.mujoco/mjpro150")
set (MUJOCO_ROOT_DIR "/home/tao/.mujoco/mjpro150")
set (Ipopt_ROOT_DIR "/usr/local")
include(FindMujoco.cmake)
include(FindIpopt.cmake)

if (NOT MUJOCO_FOUND)
	message(FATAL_ERROR "Failed to find mujoco at " ${MUJOCO_ROOT_DIR})
endif()

add_executable(simulate basic.cpp)
target_include_directories(simulate PUBLIC ${MUJOCO_INCLUDE_DIR})
target_link_libraries(simulate ${MUJOCO_LIB} ${MUJOCO_EXTERN_LIBS})

add_executable(derivative derivative.cpp)
target_include_directories(derivative PUBLIC ${MUJOCO_INCLUDE_DIR})
target_link_libraries(derivative ${MUJOCO_LIB} ${MUJOCO_EXTERN_LIBS})


# add_executable(block_opt block_opt_main.hpp block_nlp.cpp)
add_executable(traj_opt traj_opt_main.cpp)
target_include_directories(traj_opt PUBLIC ${Ipopt_INCLUDE_DIR})
target_link_libraries(traj_opt PUBLIC ${Ipopt_LIB} ${Ipopt_THIRDPARTY_LIBS})

# add_executable(hs071_nlp hs071_main.cpp hs071_nlp.cpp)
# target_include_directories(hs071_nlp PUBLIC ${Ipopt_INCLUDE_DIR})
# target_link_libraries(hs071_nlp PUBLIC ${Ipopt_LIB} ${Ipopt_THIRDPARTY_LIBS})
#
