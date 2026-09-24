# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(iDynTree)

add_subdirectory(IWearRemapper)
add_subdirectory(IAnalogSensorToIWear)
add_subdirectory(IFrameTransformToIWear)

if(ENABLE_Paexo)
    add_subdirectory(Paexo)
endif()

if(ENABLE_XsensSuit)
  add_subdirectory(XsensSuit)
endif()

if(ENABLE_ICub)
  add_subdirectory(ICub)
endif()

add_subdirectory(HumanStateProvider)
add_subdirectory(HumanWrenchProvider)
add_subdirectory(HumanDynamicsEstimator)
add_subdirectory(HumanControlBoard)
add_subdirectory(RobotPositionController)

if(HUMANSTATEPROVIDER_ENABLE_LOGGER)
    add_subdirectory(HumanLogger)
    add_subdirectory(HumanStateReplayer)
endif()
