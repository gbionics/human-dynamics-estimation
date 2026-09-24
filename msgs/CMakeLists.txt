# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

# TODO: these libraries are enforced to be static. In order to let the devices
#       find them on Windows from the build directory, their Release / Debug
#       folder should be added to the PATH. However, this does not work as expected.

# =======================
# XsensSuitControlService
# =======================

yarp_add_idl(XSENSSUITCONTROL thrift/XsensSuitControlService.thrift)

add_library(XsensSuitControl ${XSENSSUITCONTROL} thrift/XsensSuitControlService.thrift)
target_link_libraries(XsensSuitControl YARP::YARP_os)

# Extract the include directory from the files names
foreach(file ${XSENSSUITCONTROL})
    STRING(REGEX MATCH ".+\\.h?h$" file ${file})
    if(file)
        get_filename_component(include_dir ${file} DIRECTORY)
        list(APPEND XSENSSUITCONTROL_INCLUDE_DIRS ${include_dir})
        list(REMOVE_DUPLICATES XSENSSUITCONTROL_INCLUDE_DIRS)
    endif()
endforeach()

foreach(dir ${XSENSSUITCONTROL_INCLUDE_DIRS})
    get_filename_component(parent_dir_name ${dir} NAME)
    if(${parent_dir_name} STREQUAL thrift)
        list(REMOVE_ITEM XSENSSUITCONTROL_INCLUDE_DIRS ${dir})
        get_filename_component(parent_dir_path ${dir} DIRECTORY)
        list(APPEND XSENSSUITCONTROL_INCLUDE_DIRS ${parent_dir_path})
    endif()
endforeach()

# Setup the include directories
target_include_directories(XsensSuitControl PUBLIC
    $<BUILD_INTERFACE:${XSENSSUITCONTROL_INCLUDE_DIRS}>)

install(TARGETS XsensSuitControl
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

# ===========================
# HumanStateReplayerService
# ===========================

yarp_add_idl(HUMANSTATEREPLAYERSERVICE thrift/HumanStateReplayerService.thrift)

add_library(HumanStateReplayerService ${HUMANSTATEREPLAYERSERVICE} thrift/HumanStateReplayerService.thrift)
target_link_libraries(HumanStateReplayerService YARP::YARP_os)

foreach(file ${HUMANSTATEREPLAYERSERVICE})
    STRING(REGEX MATCH ".+\\.h?h$" file ${file})
    if(file)
        get_filename_component(include_dir ${file} DIRECTORY)
        list(APPEND HUMANSTATEREPLAYERSERVICE_INCLUDE_DIRS ${include_dir})
        list(REMOVE_DUPLICATES HUMANSTATEREPLAYERSERVICE_INCLUDE_DIRS)
    endif()
endforeach()

foreach(dir ${HUMANSTATEREPLAYERSERVICE_INCLUDE_DIRS})
    get_filename_component(parent_dir_name ${dir} NAME)
    if(${parent_dir_name} STREQUAL thrift)
        list(REMOVE_ITEM HUMANSTATEREPLAYERSERVICE_INCLUDE_DIRS ${dir})
        get_filename_component(parent_dir_path ${dir} DIRECTORY)
        list(APPEND HUMANSTATEREPLAYERSERVICE_INCLUDE_DIRS ${parent_dir_path})
    endif()
endforeach()

target_include_directories(HumanStateReplayerService PUBLIC
    $<BUILD_INTERFACE:${HUMANSTATEREPLAYERSERVICE_INCLUDE_DIRS}>)

install(TARGETS HumanStateReplayerService
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
