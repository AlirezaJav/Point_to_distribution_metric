cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PROJECT_OUTPUT_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/../test")
set(CMAKE_CONFIGURATION_TYPES "Release;Debug" CACHE STRING "Release;Debug")

project(pc_error)

# Comment this block out, if no multithreading is wanted. @DT
find_package(OpenMP)
if (OPENMP_FOUND)
  message(STATUS "OpenMP found.")
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  add_compile_options( -D OPENMP_FOUND ) 
endif()

###############################################################################
# Find Eigen3  This is required for point to distribution metrics
#
# This sets the following variables:
# EIGEN_FOUND - True if Eigen was found.
# EIGEN_INCLUDE_DIRS - Directories containing the Eigen include files.
# EIGEN_DEFINITIONS - Compiler flags for Eigen.
# EIGEN_VERSION - Package version

find_package(PkgConfig QUIET)
pkg_check_modules(PC_EIGEN eigen3)
set(EIGEN_DEFINITIONS ${PC_EIGEN_CFLAGS_OTHER})

find_path(EIGEN_INCLUDE_DIR Eigen/Core
    HINTS "${EIGEN_ROOT}" "$ENV{EIGEN_ROOT}" ${PC_EIGEN_INCLUDEDIR} ${PC_EIGEN_INCLUDE_DIRS}
    PATHS "$ENV{PROGRAMFILES}/Eigen" "$ENV{PROGRAMW6432}/Eigen"
          "$ENV{PROGRAMFILES}/Eigen3" "$ENV{PROGRAMW6432}/Eigen3"
    PATH_SUFFIXES eigen3 include/eigen3 include)

if(EIGEN_INCLUDE_DIR)
  file(READ "${EIGEN_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen_version_header)

  string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen_world_version_match "${_eigen_version_header}")
  set(EIGEN_WORLD_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen_major_version_match "${_eigen_version_header}")
  set(EIGEN_MAJOR_VERSION "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen_minor_version_match "${_eigen_version_header}")
  set(EIGEN_MINOR_VERSION "${CMAKE_MATCH_1}")
  set(EIGEN_VERSION ${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION})
endif()

set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Eigen DEFAULT_MSG EIGEN_INCLUDE_DIR)

mark_as_advanced(EIGEN_INCLUDE_DIR)

if(EIGEN_FOUND)
  message(STATUS "Eigen found (include: ${EIGEN_INCLUDE_DIRS}, version: ${EIGEN_VERSION})")
endif()

execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION)
if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
  # message(STATUS "Version >= 4.7!" ${GCC_VERSION})
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -std=c++11")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  # message(STATUS "Version < 4.7!" ${GCC_VERSION})
  set (CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -std=c++0x")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()


file(GLOB_RECURSE PROJ_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h*)
file(GLOB SOURCE               ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
                               ${CMAKE_SOURCE_DIR}/../dependencies/program-options-lite/* )

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}
                     ${CMAKE_SOURCE_DIR}/../dependencies/)
message(STATUS "CMAKE_SOURCE_DIR         =" ${CMAKE_SOURCE_DIR})
message(STATUS "CMAKE_CURRENT_SOURCE_DIR =" ${CMAKE_CURRENT_SOURCE_DIR})


if (MSVC)
  add_executable(pc_error ${SOURCE})
else()
  list(REMOVE_ITEM SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/stdafx.cpp")
  add_executable(pc_error ${SOURCE})
endif()

install(TARGETS pc_error DESTINATION ".")

set_target_properties(pc_error PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_OUTPUT_FOLDER})
set_target_properties(pc_error PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${PROJECT_OUTPUT_FOLDER})
set_target_properties(pc_error PROPERTIES DEBUG_POSTFIX "_d")
