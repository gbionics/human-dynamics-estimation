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

#ifndef HDE_ALGORITHMS_MINJERKTRAJGEN_HPP
#define HDE_ALGORITHMS_MINJERKTRAJGEN_HPP

#include <Eigen/Core>

#include <array>

namespace hde::algorithms
{

/**
 * Eigen-based generator of approximately minimum-jerk trajectories.
 *
 * This class reproduces the behavior of iCub::ctrl::minJerkTrajGen without
 * depending on YARP or iCub. Once constructed, init() and
 * computeNextValues() do not allocate memory when their argument size matches
 * the configured dimension.
 */
class minJerkTrajGen
{
public:
    minJerkTrajGen(unsigned int dimension, double sampleTime, double trajectoryTime);
    minJerkTrajGen(const Eigen::Ref<const Eigen::VectorXd>& initialValue,
                   double sampleTime,
                   double trajectoryTime);

    minJerkTrajGen(const minJerkTrajGen& other);
    minJerkTrajGen& operator=(const minJerkTrajGen& other);
    minJerkTrajGen(minJerkTrajGen&&) noexcept = default;
    minJerkTrajGen& operator=(minJerkTrajGen&&) noexcept = default;
    ~minJerkTrajGen() = default;

    void init(const Eigen::Ref<const Eigen::VectorXd>& initialValue);
    void computeNextValues(const Eigen::Ref<const Eigen::VectorXd>& desiredValue);

    const Eigen::VectorXd& getPos() const noexcept;
    const Eigen::VectorXd& getVel() const noexcept;
    const Eigen::VectorXd& getAcc() const noexcept;
    double getT() const noexcept;
    double getTs() const noexcept;
    bool setT(double trajectoryTime);
    bool setTs(double sampleTime);

private:
    class Filter
    {
    public:
        void configure(Eigen::Index dimension,
                       const std::array<double, 4>& numerator,
                       const std::array<double, 4>& denominator,
                       const Eigen::VectorXd& initialOutput);
        void adjustCoefficients(const std::array<double, 4>& numerator,
                                const std::array<double, 4>& denominator) noexcept;
        void initSteady(const Eigen::Ref<const Eigen::VectorXd>& output);
        void initZero(const Eigen::Ref<const Eigen::VectorXd>& nextInput);
        void filter(const Eigen::Ref<const Eigen::VectorXd>& input, Eigen::VectorXd& output);

    private:
        std::array<double, 4> m_numerator{};
        std::array<double, 4> m_denominator{};
        Eigen::VectorXd m_output;
        Eigen::MatrixXd m_previousInputs;
        Eigen::MatrixXd m_previousOutputs;
    };

    void computeCoeffs();

    unsigned int m_dimension;
    double m_sampleTime;
    double m_trajectoryTime;
    Eigen::VectorXd m_position;
    Eigen::VectorXd m_velocity;
    Eigen::VectorXd m_acceleration;
    Eigen::VectorXd m_lastReference;
    bool m_filtersInitialized{false};
    Filter m_positionFilter;
    Filter m_velocityFilter;
    Filter m_accelerationFilter;
};

} // namespace hde::algorithms

#endif // HDE_ALGORITHMS_MINJERKTRAJGEN_HPP

