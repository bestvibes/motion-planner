# - Try to find Rangev3 library installation
# The following are set after configuration is done: 
#  Rangev3_FOUND
#  Rangev3_INCLUDE_DIR
# and the following imported targets
#
#     Rangev3::Rangev3

# Download and unpack rangev3 at configure time
configure_file(${CMAKE_CURRENT_LIST_DIR}/Rangev3.cmake.in ${CMAKE_BINARY_DIR}/rangev3-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/rangev3-download )
if(result)
  message(FATAL_ERROR "CMake step for range-v3 failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/rangev3-download )
if(result)
  message(FATAL_ERROR "Build step for range-v3 failed: ${result}")
endif()

find_path(Rangev3_INCLUDE_DIR
NAMES
  "range/v3/view.hpp"
PATHS
  "${CMAKE_BINARY_DIR}/rangev3-src/include")

mark_as_advanced(Rangev3_INCLUDE_DIR)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Rangev3
  REQUIRED_VARS Rangev3_INCLUDE_DIR
)

if(Rangev3_FOUND AND NOT TARGET Rangev3::Rangev3)
    add_library(Rangev3::Rangev3 INTERFACE IMPORTED)
    set_target_properties(Rangev3::Rangev3 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${Rangev3_INCLUDE_DIR}"
    )
endif()