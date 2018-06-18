# - Try to find Rangev3 library installation
# The following are set after configuration is done: 
#  RANGEV3_FOUND
#  RANGEV3_INCLUDE_DIR
# and the following imported targets
#
#     Rangev3::Rangev3

cmake_minimum_required(VERSION 3.2)

# Download and unpack rangev3 at configure time
configure_file(${CMAKE_SOURCE_DIR}/cmake/Rangev3.cmake.in ${CMAKE_BINARY_DIR}/rangev3-download/CMakeLists.txt)
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

set(RANGEV3_FOUND ON)
set(RANGEV3_INCLUDE_DIR ${CMAKE_BINARY_DIR}/rangev3-src/include)

if(RANGEV3_FOUND AND NOT TARGET Rangev3::Rangev3)
    add_library(Rangev3::Rangev3 INTERFACE IMPORTED)
    set_target_properties(Rangev3::Rangev3 PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${RANGEV3_INCLUDE_DIR}"
    )
endif()