# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# set target name
set(EXE_TARGET_NAME HumanStateVisualizer)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

# Find required package
find_package(YARP REQUIRED)
find_package(iDynTree 12.0.0 REQUIRED)
find_package(OsqpEigen 0.4.0 REQUIRED)

# set cpp files
set(${EXE_TARGET_NAME}_SRC
  src/main.cpp)

# add include directories to the build.
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# add an executable to the project using the specified source files.
add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} )

target_link_libraries(${EXE_TARGET_NAME} LINK_PUBLIC
  YARP::YARP_os
  YARP::YARP_dev
  YARP::YARP_init
  OsqpEigen::OsqpEigen
  iDynTree::idyntree-core
  iDynTree::idyntree-modelio
  iDynTree::idyntree-visualization
  IHumanState
  IHumanStateHorizon
  IWearableTargets
  IHumanWrench
  IHumanDynamics)

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)