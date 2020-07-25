cmake_minimum_required (VERSION 3.6)

# Grab the SDK location
if(NOT DEFINED SOS_SDK_PATH)
	if(DEFINED ENV{SOS_SDK_PATH})
		set(SOS_SDK_PATH $ENV{SOS_SDK_PATH})
	elseif( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin" )
		message(WARNING "default SDK locations are deprecated")
		set(SOS_SDK_PATH /Applications/StratifyLabs-SDK)
	elseif( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows" )
		message(WARNING "default SDK locations are deprecated")
		set(SOS_SDK_PATH C:/StratifyLabs-SDK)
	elseif( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Linux" )
		message(WARNING "default SDK locations are deprecated")
		set(SOS_SDK_PATH /StratifyLabs-SDK)
	endif()
endif()
if(NOT DEFINED SOS_SDK_PATH)
	message(FATAL_ERROR "Please set environment variable SOS_SDK_PATH to location of the StratifyLabs-SDK directory")
endif()
set(SOS_TOOLCHAIN_CMAKE_PATH ${SOS_SDK_PATH}/Tools/gcc/arm-none-eabi/cmake)

include(${SOS_TOOLCHAIN_CMAKE_PATH}/sos-variables.cmake)
set(CMAKE_TOOLCHAIN_FILE ${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib-toolchain.cmake)
set(SOS_NAME sos)
project(${SOS_NAME} C ASM)

#Add sources to the project
set(SOURCES_PREFIX ${CMAKE_SOURCE_DIR}/src)
add_subdirectory(src)
file(GLOB LDSCRIPT_SOURCES "${CMAKE_SOURCE_DIR}/ldscripts/*")
set(SHARED_SOURCELIST ${SOURCES} ${LDSCRIPT_SOURCES} ${CMAKE_SOURCE_DIR}/include/imxrt_arch.h ${CMAKE_SOURCE_DIR}/README.md)

option(BUILD_ALL "Build All configurations" ON)

option(BUILD_MIMXRT1051 "Build MIMXRT1051 Configuration" OFF)
option(BUILD_MIMXRT1052 "Build All F4 configurations" OFF)

if(BUILD_MIMXRT1052 OR BUILD_ALL)
set(MIMXRT_VARIANT imxrt1052)
set(MIMXRT_VARIANT_DEFINE CPU_MIMXRT1052CVJ5B)
include(${CMAKE_SOURCE_DIR}/mimxrt1052.cmake)
endif()


file(GLOB MIMXRT1052_HEADERS ${CMAKE_SOURCE_DIR}/src/MIMXRT1052/*.h)

install(FILES ${MIMXRT1052_HEADERS} DESTINATION include/mcu/arch/imxrt/mimxrt1052)

install(DIRECTORY include/ DESTINATION include/mcu/arch/imxrt)
install(DIRECTORY ldscripts/ DESTINATION lib/ldscripts)
