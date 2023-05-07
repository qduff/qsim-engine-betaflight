cmake_minimum_required(VERSION 3.3)
project(qsim_engine)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(GEN_COVERAGE "Generate coverage profile" OFF)

#execute_process(
#        COMMAND git log -1 --format=%h
#        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external/betaflight
#        OUTPUT_VARIABLE BETAFLIGHT_GIT_HASH
#        OUTPUT_STRIP_TRAILING_WHITESPACE
#)

# Linker options for betaflight and windows
if (NOT APPLE)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T${CMAKE_SOURCE_DIR}/external/betaflight/src/main/target/SITL/pg.ld")
endif ()

if (WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wa,-mbig-obj")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libgcc -static-libstdc++")
endif ()

if(MINGW OR MSYS)
  link_libraries(ws2_32)
#   add_compile_definitions(ffs=__builtin_ffs) // only linux for windows..?
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --static")

endif()

if (GEN_COVERAGE)
    message("Building with coverage")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage")
endif ()

find_package(glm REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})


# Get betaflight sources:
add_subdirectory(external/)

set(SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/interface.cpp)

add_library(bfsitl OBJECT ${BETAFLIGHT_SOURCES} ${SOURCE_FILES})
target_compile_features(bfsitl PUBLIC cxx_std_17)

target_compile_definitions(bfsitl PUBLIC "__TARGET__=\"virtual_device\"")
target_compile_definitions(bfsitl PUBLIC "__REVISION__=\"unknown-0.0.0\"")
target_compile_definitions(bfsitl PUBLIC "_GNU_SOURCE")


target_compile_definitions(bfsitl PUBLIC SIMULATOR_BUILD)
target_compile_definitions(bfsitl PUBLIC "HSE_VALUE=8000000")
target_compile_definitions(bfsitl PUBLIC "FLASH_SIZE=2048")


target_include_directories(bfsitl PUBLIC external)
target_include_directories(bfsitl PUBLIC external/src/)
target_include_directories(bfsitl PUBLIC external/betaflight/lib/main/dyad)
target_include_directories(bfsitl PUBLIC external/betaflight/src/main)


add_subdirectory(lib/qsim-physics)



add_executable(qsim_engine src/main.cpp $<TARGET_OBJECTS:bfsitl>) # ${SOURCE_FILES}) #${BETAFLIGHT_SOURCES})
target_link_libraries(qsim_engine PUBLIC qsim-physics)
target_link_libraries(qsim_engine PUBLIC bfsitl)


#

#if (WIN32)
#  target_link_libraries(qsim_engine PUBLIC wsock32 ws2_32)
#endif (WIN32)

