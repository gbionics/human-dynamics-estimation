# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

find_package(Catch2 3 REQUIRED)
find_package(ICUB REQUIRED)

add_executable(MinJerkTrajGenTests MinJerkTrajGenTests.cpp)
target_link_libraries(MinJerkTrajGenTests
                      PRIVATE HumanDynamicsEstimation::algorithms
                              Catch2::Catch2WithMain
                              ctrlLib)

include(Catch)
catch_discover_tests(MinJerkTrajGenTests)
