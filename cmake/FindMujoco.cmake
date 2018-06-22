# - Try to find MuJoCo library installation
# The following variables are optionally searched for defaults
# MUJOCO_ROOT_DIR:            Base directory of mujoco tree to use.

# The following are set after configuration is done: 
#  MUJOCO_FOUND
#  MUJOCO_INCLUDE_DIR
#  MUJOCO_LIB
#  MUJOCO_EXTERN_LIBS
# and the following imported targets
#
#     Mujoco::Mujoco

if (NOT EXISTS ${MUJOCO_ROOT_DIR})
	message(WARNING  "MUJOCO_ROOT_DIR DOES NOT EXIST, SETTING TO /USR/LOCAL")
	set(MUJOCO_ROOT_DIR "/usr/local")
endif()
message(STATUS "MuJoCo path: " ${MUJOCO_ROOT_DIR})

set(MUJOCO_INCLUDE_DIR "${MUJOCO_ROOT_DIR}/include")
set(MUJOCO_LIB_DIR "${MUJOCO_ROOT_DIR}/bin")

find_library(MUJOCO_LIB NAMES mujoco mujoco150 PATHS ${MUJOCO_LIB_DIR})
find_library(LIB_GL NAMES GL gl OpenGL PATHS ${MUJOCO_LIB_DIR})
find_library(LIB_GLEW NAMES glew PATHS ${MUJOCO_LIB_DIR})
find_library(LIB_GLFW NAMES glfw glfw.3 PATHS ${MUJOCO_LIB_DIR})

set(MUJOCO_LIBRARIES "" CACHE STRING "Mujoco libraries" FORCE)
list(APPEND MUJOCO_LIBRARIES ${MUJOCO_LIB})
list(APPEND MUJOCO_LIBRARIES ${LIB_GL})
list(APPEND MUJOCO_LIBRARIES ${LIB_GLEW})
list(APPEND MUJOCO_LIBRARIES ${LIB_GLFW})

mark_as_advanced(MUJOCO_INCLUDE_DIR MUJOCO_LIBRARIES)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Mujoco
  REQUIRED_VARS MUJOCO_INCLUDE_DIR MUJOCO_LIBRARIES
)

if(Mujoco_FOUND AND NOT TARGET Mujoco::Mujoco)
    add_library(Mujoco::Mujoco INTERFACE IMPORTED)
    set_target_properties(Mujoco::Mujoco PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "${MUJOCO_LIBRARIES}"
    )
endif()
