# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

if(ENABLE_FrameVisualizer)
    add_subdirectory(IWearFrameVisualizer)
endif()

if(ENABLE_ForceTorqueVisualizer)
    add_subdirectory(IWearForceTorqueVisualizer)
endif()

if(HUMANSTATEPROVIDER_ENABLE_VISUALIZER)
    add_subdirectory(HumanStateVisualizer)
endif()
