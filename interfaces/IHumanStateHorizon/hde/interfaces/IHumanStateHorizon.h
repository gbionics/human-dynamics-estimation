// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_INTERFACES_IHUMANSTATEHORIZON
#define HDE_INTERFACES_IHUMANSTATEHORIZON

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace hde {
    namespace interfaces {
        class IHumanStateHorizon;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IHumanStateHorizon
{
public:
    virtual ~IHumanStateHorizon() = default;

    // Number of samples in the horizon. The samples are sorted by non-decreasing time.
    virtual size_t getNumberOfSamples() const = 0;

    // Time instant (in milliseconds) of the sample at the given horizon index.
    virtual std::int32_t getTime(size_t index) const = 0;

    virtual std::vector<std::string> getJointNames(size_t index) const = 0;
    virtual std::string getBaseName(size_t index) const = 0;
    virtual size_t getNumberOfJoints(size_t index) const = 0;

    virtual std::vector<double> getJointPositions(size_t index) const = 0;
    virtual std::vector<double> getJointVelocities(size_t index) const = 0;

    virtual std::array<double, 3> getBasePosition(size_t index) const = 0;
    virtual std::array<double, 4> getBaseOrientation(size_t index) const = 0;

    virtual std::array<double, 6> getBaseVelocity(size_t index) const = 0;

    virtual std::array<double, 3> getCoMPosition(size_t index) const = 0;
    virtual std::array<double, 3> getCoMVelocity(size_t index) const = 0;
};

#endif // HDE_INTERFACES_IHUMANSTATEHORIZON
