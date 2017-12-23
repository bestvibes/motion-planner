# - Try to find MuJoCo library installation
# The follwoing variables are optionally searched for defaults
# MUJOCO_ROOT_DIR:            Base directory of mujoco tree to use.

# The following are set after configuration is done: 
#  MUJOCO_FOUND
#  MUJOCO_INCLUDE_DIR
#  MUJOCO_LIB
#  MUJOCO_EXTERN_LIBS

message(STATUS "MuJoCo path: " ${MUJOCO_ROOT_DIR})
set(MUJOCO_FOUND ON)
if (NOT EXISTS ${MUJOCO_ROOT_DIR})
	message(WARNING  "MUJOCO_ROOT_DIR DOES NOT EXIST: " ${MUJOCO_ROOT_DIR})
	set(MUJOCO_FOUND OFF)
	return ()
endif()

set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/bin")

message(STATUS "MuJoCo lib directory: " ${MUJOCO_LIB_DIR})
file(GLOB MUJOCO_LIB ${MUJOCO_LIB_DIR}/libmujoco[0-9][0-9][0-9].so)
# find_library(MUJOCO_LIB names mujoco[0-9][0-9][0-9] PATHS ${MUJOCO_LIB_DIR})
message(STATUS "MuJoCo lib: " ${MUJOCO_LIB})
if (NOT MUJOCO_LIB) 
	message(FATAL_ERROR  "Could not find mujoco library ")
endif()

find_library(LIB_GL NAMES GL gl)
if (NOT LIB_GL) 
	message(FATAL_ERROR  "Could not find GL library ")
endif()


file(GLOB LIB_GLEW ${MUJOCO_LIB_DIR}/libglew.so)
if (NOT LIB_GLEW) 
	message(FATAL_ERROR  "Could not find GLEW library ")
else()
	message(STATUS  "find glew library at: " ${LIB_GLEW})
endif()


# find_library(LIB_GLFW name glfw)
file(GLOB LIB_GLFW ${MUJOCO_LIB_DIR}/libglfw.so.[0-9])
# find_library(MUJOCO_LIB names mujoco[0-9][0-9][0-9] PATHS ${MUJOCO_LIB_DIR})
if (NOT LIB_GLFW) 
	message(FATAL_ERROR  "Could not find GLFW library ")
else()
	message(STATUS  "find GLFW library at: " ${LIB_GLFW})
endif()

set(MUJOCO_EXTERN_LIBS ${LIB_GL} ${LIB_GLEW} ${LIB_GLFW})
