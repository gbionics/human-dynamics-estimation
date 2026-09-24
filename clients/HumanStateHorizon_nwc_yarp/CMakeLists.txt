# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

yarp_prepare_plugin(human_state_horizon_nwc_yarp
    TYPE hde::devices::HumanStateHorizon_nwc_yarp
    INCLUDE HumanStateHorizon_nwc_yarp.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(HumanStateHorizon_nwc_yarp
    HumanStateHorizon_nwc_yarp.cpp
    HumanStateHorizon_nwc_yarp.h)

target_include_directories(HumanStateHorizon_nwc_yarp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_link_libraries(HumanStateHorizon_nwc_yarp PUBLIC
    IHumanStateHorizon
    trintrin::msgs
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init)

yarp_install(
    TARGETS HumanStateHorizon_nwc_yarp
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
