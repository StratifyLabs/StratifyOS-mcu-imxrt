cmake_minimum_required (VERSION 3.6)

#Add sources to the project
set(SOURCES_PREFIX ${CMAKE_SOURCE_DIR}/src)
add_subdirectory(src)
file(GLOB LDSCRIPT_SOURCES "${CMAKE_SOURCE_DIR}/ldscripts/*")
set(SHARED_SOURCELIST ${SOURCES} ${LDSCRIPT_SOURCES} ${CMAKE_SOURCE_DIR}/include/imxrt_arch.h ${CMAKE_SOURCE_DIR}/README.md)

if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Darwin" )
  set(SOS_TOOLCHAIN_CMAKE_PATH /Applications/StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()
if( ${CMAKE_HOST_SYSTEM_NAME} STREQUAL "Windows" )
  set(SOS_TOOLCHAIN_CMAKE_PATH C:/StratifyLabs-SDK/Tools/gcc/arm-none-eabi/cmake)
endif()
set(CMAKE_TOOLCHAIN_FILE ${SOS_TOOLCHAIN_CMAKE_PATH}/sos-lib-toolchain.cmake)
set(SOS_LIB_NAME sos)
project(${SOS_LIB_NAME} C ASM)

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
