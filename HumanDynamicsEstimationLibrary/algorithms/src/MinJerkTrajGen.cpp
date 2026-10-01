/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
 */

// SPDX-License-Identifier: BSD-3-Clause
// Derived from iCub ctrlLib's minJerkCtrl and Filter implementations.

#include <hde/algorithms/MinJerkTrajGen.hpp>

#include <cassert>

using hde::algorithms::MinJerkTrajGen;

void MinJerkTrajGen::Filter::configure(Eigen::Index dimension,
                                       const std::array<double, 4>& numerator,
                                       const std::array<double, 4>& denominator,
                                       const Eigen::VectorXd& initialOutput)
{
    m_numerator = numerator;
    m_denominator = denominator;
    m_output.resize(dimension);
    m_previousInputs.resize(dimension, 3);
    m_previousOutputs.resize(dimension, 3);
    m_previousInputs.setZero();
    m_previousOutputs.setZero();
    initSteady(initialOutput);
}

void MinJerkTrajGen::Filter::adjustCoefficients(
    const std::array<double, 4>& numerator,
    const std::array<double, 4>& denominator) noexcept
{
    m_numerator = numerator;
    m_denominator = denominator;
}

void MinJerkTrajGen::Filter::initSteady(const Eigen::Ref<const Eigen::VectorXd>& output)
{
    assert(output.size() == m_output.size());
    double numeratorSum = 0.0;
    double denominatorSum = 0.0;
    for (std::size_t i = 0; i < 4; ++i) {
        numeratorSum += m_numerator[i];
        denominatorSum += m_denominator[i];
    }

    m_output = output;
    for (Eigen::Index history = 0; history < 3; ++history) {
        for (Eigen::Index element = 0; element < output.size(); ++element) {
            m_previousOutputs(element, history) = output[element];
            m_previousInputs(element, history) =
                (denominatorSum / numeratorSum) * output[element];
        }
    }
}

void MinJerkTrajGen::Filter::initZero(const Eigen::Ref<const Eigen::VectorXd>& nextInput)
{
    assert(nextInput.size() == m_output.size());
    m_output.setZero();
    m_previousOutputs.setZero();
    for (Eigen::Index history = 0; history < 3; ++history) {
        for (Eigen::Index element = 0; element < nextInput.size(); ++element) {
            m_previousInputs(element, history) = nextInput[element];
        }
    }
}

void MinJerkTrajGen::Filter::filter(const Eigen::Ref<const Eigen::VectorXd>& input,
                                    Eigen::VectorXd& output)
{
    assert(input.size() == m_output.size());
    assert(output.size() == m_output.size());
    for (Eigen::Index element = 0; element < input.size(); ++element) {
        m_output[element] = m_numerator[0] * input[element];
    }
    for (Eigen::Index history = 0; history < 3; ++history) {
        for (Eigen::Index element = 0; element < input.size(); ++element) {
            m_output[element] += m_numerator[static_cast<std::size_t>(history + 1)]
                                 * m_previousInputs(element, history);
        }
    }
    for (Eigen::Index history = 0; history < 3; ++history) {
        for (Eigen::Index element = 0; element < input.size(); ++element) {
            m_output[element] -= m_denominator[static_cast<std::size_t>(history + 1)]
                                 * m_previousOutputs(element, history);
        }
    }
    for (Eigen::Index element = 0; element < input.size(); ++element) {
        m_output[element] /= m_denominator[0];
    }
    for (Eigen::Index history = 2; history > 0; --history) {
        for (Eigen::Index element = 0; element < input.size(); ++element) {
            m_previousInputs(element, history) = m_previousInputs(element, history - 1);
            m_previousOutputs(element, history) = m_previousOutputs(element, history - 1);
        }
    }
    for (Eigen::Index element = 0; element < input.size(); ++element) {
        m_previousInputs(element, 0) = input[element];
        m_previousOutputs(element, 0) = m_output[element];
        output[element] = m_output[element];
    }
}

MinJerkTrajGen::MinJerkTrajGen(unsigned int dimension,
                               double sampleTime,
                               double trajectoryTime)
    : m_dimension(dimension)
    , m_sampleTime(sampleTime)
    , m_trajectoryTime(trajectoryTime)
    , m_position(Eigen::VectorXd::Zero(dimension))
    , m_velocity(Eigen::VectorXd::Zero(dimension))
    , m_acceleration(Eigen::VectorXd::Zero(dimension))
    , m_lastReference(Eigen::VectorXd::Zero(dimension))
{
    computeCoeffs();
}

MinJerkTrajGen::MinJerkTrajGen(const Eigen::Ref<const Eigen::VectorXd>& initialValue,
                               double sampleTime,
                               double trajectoryTime)
    : m_dimension(static_cast<unsigned int>(initialValue.size()))
    , m_sampleTime(sampleTime)
    , m_trajectoryTime(trajectoryTime)
    , m_position(initialValue)
    , m_velocity(Eigen::VectorXd::Zero(initialValue.size()))
    , m_acceleration(Eigen::VectorXd::Zero(initialValue.size()))
    , m_lastReference(initialValue)
{
    computeCoeffs();
}

MinJerkTrajGen::MinJerkTrajGen(const MinJerkTrajGen& other)
    : m_dimension(other.m_dimension)
    , m_sampleTime(other.m_sampleTime)
    , m_trajectoryTime(other.m_trajectoryTime)
    , m_position(other.m_position)
    , m_velocity(other.m_velocity)
    , m_acceleration(other.m_acceleration)
    , m_lastReference(other.m_lastReference)
{
    computeCoeffs();
}

MinJerkTrajGen& MinJerkTrajGen::operator=(const MinJerkTrajGen& other)
{
    if (this != &other) {
        m_dimension = other.m_dimension;
        m_sampleTime = other.m_sampleTime;
        m_trajectoryTime = other.m_trajectoryTime;
        m_position = other.m_position;
        m_velocity = other.m_velocity;
        m_acceleration = other.m_acceleration;
        m_lastReference = other.m_lastReference;
        m_filtersInitialized = false;
        computeCoeffs();
    }
    return *this;
}

void MinJerkTrajGen::init(const Eigen::Ref<const Eigen::VectorXd>& initialValue)
{
    assert(initialValue.size() == static_cast<Eigen::Index>(m_dimension));
    m_lastReference = initialValue;
    m_position = initialValue;
    m_positionFilter.initSteady(initialValue);
    m_velocityFilter.initZero(initialValue);
    m_accelerationFilter.initZero(initialValue);
}

void MinJerkTrajGen::computeNextValues(const Eigen::Ref<const Eigen::VectorXd>& desiredValue)
{
    assert(desiredValue.size() == static_cast<Eigen::Index>(m_dimension));
    m_lastReference = desiredValue;
    m_positionFilter.filter(desiredValue, m_position);
    m_velocityFilter.filter(desiredValue, m_velocity);
    m_accelerationFilter.filter(desiredValue, m_acceleration);
}

const Eigen::VectorXd& MinJerkTrajGen::getPos() const noexcept { return m_position; }
const Eigen::VectorXd& MinJerkTrajGen::getVel() const noexcept { return m_velocity; }
const Eigen::VectorXd& MinJerkTrajGen::getAcc() const noexcept { return m_acceleration; }
double MinJerkTrajGen::getT() const noexcept { return m_trajectoryTime; }
double MinJerkTrajGen::getTs() const noexcept { return m_sampleTime; }

bool MinJerkTrajGen::setT(double trajectoryTime)
{
    if (trajectoryTime <= 0.0) {
        return false;
    }
    m_trajectoryTime = trajectoryTime;
    computeCoeffs();
    return true;
}

bool MinJerkTrajGen::setTs(double sampleTime)
{
    if (sampleTime <= 0.0) {
        return false;
    }
    m_sampleTime = sampleTime;
    computeCoeffs();
    return true;
}

void MinJerkTrajGen::computeCoeffs()
{
    // 90% of steady-state value in t=T
    // transient extinguished for t>=1.5*T
    const double a = -150.765868956161
                     / (m_trajectoryTime * m_trajectoryTime * m_trajectoryTime);
    const double b = -84.9812819469538 / (m_trajectoryTime * m_trajectoryTime);
    const double c = -15.9669610709384 / m_trajectoryTime;

    // implementing F(s)=-a/(s^3-c*s^2-b*s-a)
    const double m = 4.0 * c * m_sampleTime;
    const double n = 2.0 * b * m_sampleTime * m_sampleTime;
    double p = a * m_sampleTime * m_sampleTime * m_sampleTime;
    const std::array<double, 4> positionNumerator{p, 3.0 * p, 3.0 * p, p};
    const std::array<double, 4> denominator{m + n + p - 8.0,
                                           -m + n + 3.0 * p + 24.0,
                                           -m - n + 3.0 * p - 24.0,
                                           m - n + p + 8.0};

    // implementing F(s)=-a*s/(s^3-c*s^2-b*s-a)
    p = 2.0 * a * m_sampleTime * m_sampleTime;
    const std::array<double, 4> velocityNumerator{p, p, -p, -p};

    // implementing F(s)=-a*s^2/(s^3-c*s^2-b*s-a)
    p = 4.0 * a * m_sampleTime;
    const std::array<double, 4> accelerationNumerator{p, -p, -p, p};

    if (!m_filtersInitialized) {
        m_positionFilter.configure(m_dimension, positionNumerator, denominator, m_position);
        m_velocityFilter.configure(m_dimension, velocityNumerator, denominator, m_velocity);
        m_accelerationFilter.configure(m_dimension, accelerationNumerator, denominator, m_acceleration);
    } else {
        m_positionFilter.adjustCoefficients(positionNumerator, denominator);
        m_velocityFilter.adjustCoefficients(velocityNumerator, denominator);
        m_accelerationFilter.adjustCoefficients(accelerationNumerator, denominator);
    }
    m_filtersInitialized = true;
    m_velocityFilter.initZero(m_position);
    m_accelerationFilter.initZero(m_position);
}
