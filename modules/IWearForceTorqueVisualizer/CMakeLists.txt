# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

set(EXE_TARGET_NAME IWearForceTorqueVisualizerModule)

find_package(YARP REQUIRED)
find_package(iDynTree REQUIRED)

add_executable(${EXE_TARGET_NAME} src/main.cpp)

target_link_libraries(${EXE_TARGET_NAME} PUBLIC
    IWear
    YARP::YARP_os
    YARP::YARP_sig
    YARP::YARP_dev
    YARP::YARP_math
    iDynTree::idyntree-visualization
    )

install(TARGETS ${EXE_TARGET_NAME} DESTINATION bin)

file(GLOB TARGET_INI_FILES conf/*.ini)

install(FILES ${TARGET_INI_FILES}
        DESTINATION ${CMAKE_INSTALL_DATADIR}/HumanDynamicsEstimation)
