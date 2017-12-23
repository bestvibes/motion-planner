# - Try to find Ipopt library installation
# The follwoing variables are optionally searched for defaults
# Ipopt_ROOT_DIR:            Base directory of Ipopt tree to use.

# The following are set after configuration is done: 
#  Ipopt_FOUND
#  Ipopt_INCLUDE_DIR
#  Ipopt_THIRDPARTY_INCLUDE_DIR
#  Ipopt_LIB
#  Ipopt_THIRDPARTY_LIBS

message(STATUS "Ipopt path: " ${Ipopt_ROOT_DIR})
set(Ipopt_FOUND ON)
if (NOT EXISTS ${Ipopt_ROOT_DIR})
	message(WARNING  "Ipopt_ROOT_DIR DOES NOT EXIST: " ${Ipopt_ROOT_DIR})
	set(Ipopt_FOUND OFF)
	return ()
endif()

set(Ipopt_INCLUDE_DIR "${Ipopt_ROOT_DIR}/include/coin")
set(Ipopt_LIB_DIR "${Ipopt_ROOT_DIR}/lib")
set(Ipopt_THIRDPARTY_INCLUDE_DIR "${Ipopt_INCLUDE_DIR}/ThirdParty")

find_library(Ipopt_LIB NAMES ipopt PATHS Ipopt_LIB)
if (NOT Ipopt_LIB)
	message(WARNING  "Can not find Ipopt at : " ${Ipopt_LIB_DIR})
else()
	message(STATUS  "find Ipopt lib at : " ${Ipopt_LIB})
endif()

# set(THIRDPARTY_LIB_NAMES "")
file(GLOB Ipopt_THIRDPARTY_LIBS ${Ipopt_LIB_DIR}/libcoin*.so)
foreach(var ${Ipopt_THIRDPARTY_LIBS})
	message(STATUS "Ipopt ThirdParty lib: " ${var})
endforeach()

