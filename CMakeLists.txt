# Copyright: (C) 2018 Sapienza University of Rome
# Author: Luigi Penco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.3)

project(mpc_engine)

set(MPCEngine_VERSION 1.0.0)

include(FindPackageHandleStandardArgs)
include(GenerateExportHeader)
include(CMakePackageConfigHelpers)

set(EXECUTABLE_OUTPUT_PATH ../bin/)
## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++17)

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
endif()

macro(green_message [args])
  message("${BoldGreen}" ${ARGV0} "${ColourReset}")
endmacro(green_message)

macro(yellow_message [args])
  message("${BoldYellow}" ${ARGV0} "${ColourReset}")
endmacro(yellow_message)

#eigen include headers
set ( EIGEN3_INCLUDE_DIRS  "/usr/include/eigen3" )
include_directories ( ${EIGEN3_INCLUDE_DIRS} )

# qpoases include directories and link directories
include_directories(${CMAKE_SOURCE_DIR}/dependencies/qpOASES/include)
link_directories(${CMAKE_SOURCE_DIR}/dependencies/qpOASES/bin)

include_directories(./include)

#set(MPCEngine_LIBRARIES 
  #${qpOASES_LIBRARIES}
#  Eigen3::Eigen
#)

set(MPCEngine_current_func_HDR
  ${CMAKE_SOURCE_DIR}/include/current_func/compute_A.h
  ${CMAKE_SOURCE_DIR}/include/current_func/compute_H.h
  ${CMAKE_SOURCE_DIR}/include/current_func/compute_g.h
  ${CMAKE_SOURCE_DIR}/include/current_func/compute_ub.h
)

install(FILES ${MPCEngine_current_func_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/MPCEngine/current_func/)


set(MPCEngine_Solvers_HDR
  ${CMAKE_SOURCE_DIR}/include/solvers/AbsSolver.hpp
  ${CMAKE_SOURCE_DIR}/include/solvers/all_solvers.hpp
  ${CMAKE_SOURCE_DIR}/include/solvers/qpOASESFunction.h
  ${CMAKE_SOURCE_DIR}/include/solvers/qpoasesSolver.hpp
)

install(FILES ${MPCEngine_Solvers_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/MPCEngine/solvers/)

set(MPCEngine_Api_HDR
  ${CMAKE_SOURCE_DIR}/include/api/MPCproblem.hpp
  ${CMAKE_SOURCE_DIR}/include/api/MPCregulator.hpp
  ${CMAKE_SOURCE_DIR}/include/api/MPCtracker.hpp
)

install(FILES ${MPCEngine_Api_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/MPCEngine/api/)

set(MPCEngine_Trajectories_HDR
  ${CMAKE_SOURCE_DIR}/include/trajectories/all_traj.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/all_traj_include.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/recorded_f.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/sinusoidal_f.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/trajectories.hpp
)

install(FILES ${MPCEngine_Trajectories_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/MPCEngine/trajectories/)

set(MPCEngine_TestEnv_HDR
  ${CMAKE_SOURCE_DIR}/include/test_env/abstract_env.hpp
  ${CMAKE_SOURCE_DIR}/include/test_env/all_env.hpp
  ${CMAKE_SOURCE_DIR}/include/test_env/cart_pole.hpp
  ${CMAKE_SOURCE_DIR}/include/test_env/twoRRobot.hpp
)

install(FILES ${MPCEngine_TestEnv_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/MPCEngine/test_env/)

install(FILES ${CMAKE_SOURCE_DIR}/include/mpc_engine
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/MPCEngine/
)

set(MPCEngine_SRC
  ${CMAKE_SOURCE_DIR}/src/current_func/compute_A.cpp
  ${CMAKE_SOURCE_DIR}/src/current_func/compute_H.cpp
  ${CMAKE_SOURCE_DIR}/src/current_func/compute_g.cpp
  ${CMAKE_SOURCE_DIR}/src/current_func/compute_ub.cpp
  ${CMAKE_SOURCE_DIR}/src/solvers/qpoasesSolver.cpp
  ${CMAKE_SOURCE_DIR}/src/api/MPCregulator.cpp
  ${CMAKE_SOURCE_DIR}/src/api/MPCtracker.cpp
)

set(MPCEngine_HDR
  ${CMAKE_SOURCE_DIR}/include/solvers/AbsSolver.hpp
  ${CMAKE_SOURCE_DIR}/include/solvers/all_solvers.hpp
  ${CMAKE_SOURCE_DIR}/include/current_func/compute_A.h
  ${CMAKE_SOURCE_DIR}/include/current_func/compute_H.h
  ${CMAKE_SOURCE_DIR}/include/current_func/compute_g.h
  ${CMAKE_SOURCE_DIR}/include/current_func/compute_ub.h
  ${CMAKE_SOURCE_DIR}/include/solvers/qpOASESFunction.h
  ${CMAKE_SOURCE_DIR}/include/solvers/qpoasesSolver.hpp
  ${CMAKE_SOURCE_DIR}/include/api/MPCproblem.hpp
  ${CMAKE_SOURCE_DIR}/include/api/MPCregulator.hpp
  ${CMAKE_SOURCE_DIR}/include/api/MPCtracker.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/all_traj.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/all_traj_include.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/recorded_f.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/sinusoidal_f.hpp
  ${CMAKE_SOURCE_DIR}/include/trajectories/trajectories.hpp
  ${CMAKE_SOURCE_DIR}/include/test_env/abstract_env.hpp
  ${CMAKE_SOURCE_DIR}/include/test_env/all_env.hpp
  ${CMAKE_SOURCE_DIR}/include/test_env/cart_pole.hpp
  ${CMAKE_SOURCE_DIR}/include/test_env/twoRRobot.hpp
)

add_library(MPCEngine SHARED ${MPCEngine_SRC} ${MPCEngine_HDR})
#target_link_libraries(MPCEngine ${MPCEngine_LIBRARIES}qpOASES)
target_link_libraries(MPCEngine qpOASES)



generate_export_header(MPCEngine)
set_property(TARGET MPCEngine PROPERTY VERSION ${MPCEngine_VERSION})
set_property(TARGET MPCEngine PROPERTY
  INTERFACE_MPCEngine_MAJOR_VERSION 1.0.0)
set_property(TARGET MPCEngine APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING MPCEngine_MAJOR_VERSION
)

install(TARGETS MPCEngine EXPORT MPCEngineTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/MPCEngine
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/MPCEngine
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/MPCEngine
  INCLUDES DESTINATION ${CMAKE_INSTALL_PREFIX}/include/MPCEngine
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/MPCEngineConfigVersion.cmake
  VERSION ${MPCEngine_VERSION}
  COMPATIBILITY AnyNewerVersion
)

export(EXPORT MPCEngineTargets
  FILE ${CMAKE_CURRENT_BINARY_DIR}/MPCEngine/MPCEngineTargets.cmake
  NAMESPACE MPCEngine::
)

set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/include/MPCEngine)
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/lib/MPCEngine/)
set(INSTALL_LIBRARIES
  ${CMAKE_INSTALL_PREFIX}/lib/MPCEngine/libMPCEngine.so
  ${qpOASES_LIBRARIES}
)

configure_package_config_file(cmake/MPCEngineConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/MPCEngineConfig.cmake
  INSTALL_DESTINATION ${LIBRARY_INSTALL_DIR}/cmake
  PATH_VARS INCLUDE_INSTALL_DIR LIBRARY_INSTALL_DIR INSTALL_LIBRARIES
)

install(EXPORT MPCEngineTargets
  FILE
    MPCEngineTargets.cmake
  NAMESPACE
    MPCEngine::
  DESTINATION
    ${LIBRARY_INSTALL_DIR}/cmake
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/MPCEngineConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/MPCEngineConfigVersion.cmake
  DESTINATION
    ${LIBRARY_INSTALL_DIR}/cmake
  COMPONENT
    Devel
)
