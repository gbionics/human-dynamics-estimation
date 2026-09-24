# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

add_library(IHumanStateHorizon INTERFACE)

target_sources(IHumanStateHorizon INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/hde/interfaces/IHumanStateHorizon.h>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces/IHumanStateHorizon.h>)

target_include_directories(IHumanStateHorizon INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces>)

install(
    FILES hde/interfaces/IHumanStateHorizon.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/hde/interfaces)
