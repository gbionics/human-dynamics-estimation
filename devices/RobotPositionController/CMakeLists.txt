# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(iDynTree REQUIRED)

yarp_prepare_plugin(robot_position_controller
    TYPE hde::devices::RobotPositionController
    INCLUDE RobotPositionController.h
    CATEGORY device
    ADVANCED
    DEFAULT ON)

yarp_add_plugin(RobotPositionController
    RobotPositionController.cpp
    RobotPositionController.h)

target_include_directories(RobotPositionController PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

add_definitions(-D_USE_MATH_DEFINES)

target_link_libraries(RobotPositionController PUBLIC
    IHumanState
    YARP::YARP_os
    YARP::YARP_dev
    YARP::YARP_init
    iDynTree::idyntree-model
    HumanDynamicsEstimation::algorithms)

yarp_install(
    TARGETS RobotPositionController
    COMPONENT runtime
    LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
    ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
    YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})
