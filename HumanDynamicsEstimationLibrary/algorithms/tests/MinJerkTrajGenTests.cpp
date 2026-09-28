// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <hde/algorithms/MinJerkTrajGen.hpp>

#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/sig/Vector.h>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace
{

constexpr double AbsoluteTolerance = 1.0e-11;
constexpr double RelativeTolerance = 2.0e-11;

yarp::sig::Vector toYarp(const Eigen::VectorXd& input)
{
    yarp::sig::Vector output(static_cast<std::size_t>(input.size()));
    std::copy(input.data(), input.data() + input.size(), output.data());
    return output;
}

void requireEqual(const Eigen::VectorXd& actual,
                  const yarp::sig::Vector& expected,
                  const std::string& quantity,
                  int sample)
{
    REQUIRE(actual.size() == static_cast<Eigen::Index>(expected.size()));
    for (Eigen::Index i = 0; i < actual.size(); ++i) {
        CAPTURE(quantity, sample, i, actual[i], expected[static_cast<std::size_t>(i)]);
        REQUIRE(actual[i]
                == Catch::Approx(expected[static_cast<std::size_t>(i)])
                       .margin(AbsoluteTolerance)
                       .epsilon(RelativeTolerance));
    }
}

void requireGeneratorsEqual(const hde::algorithms::minJerkTrajGen& actual,
                            const iCub::ctrl::minJerkTrajGen& expected,
                            int sample)
{
    requireEqual(actual.getPos(), expected.getPos(), "position", sample);
    requireEqual(actual.getVel(), expected.getVel(), "velocity", sample);
    requireEqual(actual.getAcc(), expected.getAcc(), "acceleration", sample);
}

Eigen::VectorXd referenceAt(int sample, Eigen::Index dimension)
{
    Eigen::VectorXd reference(dimension);
    for (Eigen::Index i = 0; i < dimension; ++i) {
        const double phase = 0.013 * sample * (i + 1);
        reference[i] = 25.0 * std::sin(phase) + 7.0 * std::cos(0.37 * phase)
                       + ((sample / 71) % 3 - 1) * (i + 0.25);
    }
    return reference;
}

} // namespace

TEST_CASE("Eigen minimum-jerk generator matches iCub over changing references")
{
    struct Parameters
    {
        unsigned int dimension;
        double sampleTime;
        double trajectoryTime;
    };
    const std::array<Parameters, 4> cases{{{1, 0.001, 0.15},
                                           {3, 0.01, 1.0},
                                           {7, 0.02, 2.5},
                                           {12, 0.005, 0.75}}};

    for (const auto& parameters : cases) {
        CAPTURE(parameters.dimension, parameters.sampleTime, parameters.trajectoryTime);
        Eigen::VectorXd initial(parameters.dimension);
        for (Eigen::Index i = 0; i < initial.size(); ++i) {
            initial[i] = -12.0 + 2.75 * i;
        }

        hde::algorithms::minJerkTrajGen actual(initial,
                                               parameters.sampleTime,
                                               parameters.trajectoryTime);
        iCub::ctrl::minJerkTrajGen expected(toYarp(initial),
                                            parameters.sampleTime,
                                            parameters.trajectoryTime);
        requireGeneratorsEqual(actual, expected, -1);

        for (int sample = 0; sample < 1200; ++sample) {
            Eigen::VectorXd reference = referenceAt(sample, parameters.dimension);
            actual.computeNextValues(reference);
            expected.computeNextValues(toYarp(reference));
            requireGeneratorsEqual(actual, expected, sample);
        }
    }
}

TEST_CASE("Initialization and run-time parameter changes match iCub")
{
    constexpr unsigned int Dimension = 5;
    hde::algorithms::minJerkTrajGen actual(Dimension, 0.01, 1.2);
    iCub::ctrl::minJerkTrajGen expected(Dimension, 0.01, 1.2);

    Eigen::VectorXd initial(Dimension);
    initial << -40.0, -3.5, 0.0, 12.25, 90.0;
    actual.init(initial);
    expected.init(toYarp(initial));
    requireGeneratorsEqual(actual, expected, -1);

    const double* positionStorage = actual.getPos().data();
    const double* velocityStorage = actual.getVel().data();
    const double* accelerationStorage = actual.getAcc().data();

    for (int sample = 0; sample < 700; ++sample) {
        if (sample == 83) {
            REQUIRE(actual.setT(0.35) == expected.setT(0.35));
        } else if (sample == 219) {
            REQUIRE(actual.setTs(0.004) == expected.setTs(0.004));
        } else if (sample == 401) {
            REQUIRE(actual.setT(2.75) == expected.setT(2.75));
            REQUIRE(actual.setTs(0.015) == expected.setTs(0.015));
        }

        Eigen::VectorXd reference = referenceAt(sample, Dimension);
        actual.computeNextValues(reference);
        expected.computeNextValues(toYarp(reference));
        requireGeneratorsEqual(actual, expected, sample);

        REQUIRE(actual.getPos().data() == positionStorage);
        REQUIRE(actual.getVel().data() == velocityStorage);
        REQUIRE(actual.getAcc().data() == accelerationStorage);
    }

    REQUIRE_FALSE(actual.setT(0.0));
    REQUIRE_FALSE(expected.setT(0.0));
    REQUIRE_FALSE(actual.setTs(-1.0));
    REQUIRE_FALSE(expected.setTs(-1.0));
    REQUIRE(actual.getT() == Catch::Approx(expected.getT()));
    REQUIRE(actual.getTs() == Catch::Approx(expected.getTs()));
}

TEST_CASE("Copy construction and assignment have iCub-compatible reset semantics")
{
    Eigen::VectorXd initial(4);
    initial << 1.0, -2.0, 3.0, -4.0;
    hde::algorithms::minJerkTrajGen actual(initial, 0.01, 0.8);
    iCub::ctrl::minJerkTrajGen expected(toYarp(initial), 0.01, 0.8);

    for (int sample = 0; sample < 100; ++sample) {
        const Eigen::VectorXd reference = referenceAt(sample, initial.size());
        actual.computeNextValues(reference);
        expected.computeNextValues(toYarp(reference));
    }

    hde::algorithms::minJerkTrajGen actualCopy(actual);
    iCub::ctrl::minJerkTrajGen expectedCopy(expected);
    hde::algorithms::minJerkTrajGen actualAssigned(1, 0.1, 1.0);
    iCub::ctrl::minJerkTrajGen expectedAssigned(1, 0.1, 1.0);
    actualAssigned = actual;
    expectedAssigned = expected;

    requireGeneratorsEqual(actualCopy, expectedCopy, -1);
    requireGeneratorsEqual(actualAssigned, expectedAssigned, -1);
    for (int sample = 100; sample < 300; ++sample) {
        const Eigen::VectorXd reference = referenceAt(sample, initial.size());
        const yarp::sig::Vector yarpReference = toYarp(reference);
        actualCopy.computeNextValues(reference);
        expectedCopy.computeNextValues(yarpReference);
        actualAssigned.computeNextValues(reference);
        expectedAssigned.computeNextValues(yarpReference);
        requireGeneratorsEqual(actualCopy, expectedCopy, sample);
        requireGeneratorsEqual(actualAssigned, expectedAssigned, sample);
    }
}
