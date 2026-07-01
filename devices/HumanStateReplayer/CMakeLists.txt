# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(YARP COMPONENTS dev REQUIRED)
find_package(matioCpp REQUIRED)

yarp_prepare_plugin(human_state_replayer
    TYPE hde::devices::HumanStateReplayer
    INCLUDE HumanStateReplayer.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanStateReplayer
    HumanStateReplayer.cpp
    HumanStateReplayer.h)

target_include_directories(HumanStateReplayer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanStateReplayer PUBLIC
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init
    matioCpp::matioCpp
    trintrin::msgs
    IHumanState
    HumanStateReplayerService)

yarp_install(
    TARGETS HumanStateReplayer
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
