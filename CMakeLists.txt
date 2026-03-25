# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(HumanDynamicsEstimation
        LANGUAGES CXX
        VERSION 5.0.1)

# =====================
# PROJECT CONFIGURATION
# =====================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(BUILD_DOCS)
    add_subdirectory(doc)
    return()
endif()

include(GNUInstallDirs)
include(CTest)

option(HUMANSTATEPROVIDER_ENABLE_VISUALIZER "Enable the iDyntree-irricht based Human State Visualizer" OFF)

find_package(robometry QUIET)
option(HUMANSTATEPROVIDER_ENABLE_LOGGER "Enable the robometry based Human Logger" ${robometry_FOUND})

option(BUILD_TESTING "Build tests" OFF)

# Find packages needed for this main CMakeLists.txt.
# Subfolders will find the dependencies they need.
find_package(YARP 3.2 REQUIRED)
find_package(YCM REQUIRED)
find_package(trintrin REQUIRED)

# Import cmake utilities
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(AddWarningsConfigurationToTarget)

# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Shared/Dynamic or Static library?
option(BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Plugins are force to be Shared/Dynamic Library
set(YARP_FORCE_DYNAMIC_PLUGINS ON)

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

if(NOT MSVC)
    option(HDE_ENABLE_RPATH "Enable rpath" TRUE)
    mark_as_advanced(HDE_ENABLE_RPATH)
    include(AddInstallRPATHSupport)
    add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
                              LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
                              DEPENDS HDE_ENABLE_RPATH
                              USE_LINK_PATH)
endif()

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
    if(${LINKER_BIN} STREQUAL "ld")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--unresolved-symbols=report-all")
    endif()
endif()

if(WIN32)
  #Add compile definition to suppress warning related to header <experimental/filesystem>
  add_compile_definitions(_SILENCE_EXPERIMENTAL_FILESYSTEM_DEPRECATION_WARNING)
endif()

# Flag to enable XSensSuit wearable device
find_package(XsensXME QUIET)
option(ENABLE_XsensSuit "Flag that enables building XsensSuit wearable device" ${XsensXME_FOUND})
if(ENABLE_XsensSuit)
  add_subdirectory(XSensMVN)
endif()

# Flag to enable Paexo wearable device
option(ENABLE_Paexo "Flag that enables building Paexo wearable device" OFF)

# Flag to enable IWearFrameVisualizer module
find_package(iDynTree QUIET)
option(ENABLE_FrameVisualizer "Flag that enables building IWearFrameVisualizer module" ${iDynTree_FOUND})

# Flag to enable IWearLogger device
find_package(robometry QUIET)
option(ENABLE_Logger "Flag that enables building Wearable Logger device" ${robometry_FOUND})

# Flag to enable ICub wearable device
option(ENABLE_ICub "Flag that enables building iCub wearable device" ${iDynTree_FOUND})


# Add the uninstall target
include(AddUninstallTarget)

# ===================
# PROJECT DIRECTORIES
# ===================
include(InstallBasicPackageFiles)

yarp_configure_plugins_installation(hde)
add_subdirectory(conf)
add_subdirectory(msgs)
add_subdirectory(interfaces)
add_subdirectory(impl)
add_subdirectory(devices)
add_subdirectory(wrappers)
add_subdirectory(app)
add_subdirectory(modules)
add_subdirectory(servers)
add_subdirectory(clients)
add_subdirectory(HumanDynamicsEstimationLibrary)

install_basic_package_files(${PROJECT_NAME}
  VERSION ${${PROJECT_NAME}_VERSION}
  COMPATIBILITY AnyNewerVersion
  VARS_PREFIX ${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
  DEPENDENCIES YARP trintrin)

# Add the uninstall target
include(AddUninstallTarget)
