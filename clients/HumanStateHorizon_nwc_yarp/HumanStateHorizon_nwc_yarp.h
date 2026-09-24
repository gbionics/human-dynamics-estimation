// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANSTATEHORIZON_NWC_YARP
#define HDE_DEVICES_HUMANSTATEHORIZON_NWC_YARP

#include <hde/interfaces/IHumanStateHorizon.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IPreciselyTimed.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/TypedReaderCallback.h>

#include <memory>

namespace trintrin::msgs {
    class HumanStateHorizon;
}
namespace hde::devices {
    class HumanStateHorizon_nwc_yarp;
} // namespace hde::devices

class hde::devices::HumanStateHorizon_nwc_yarp final
    : public yarp::dev::DeviceDriver
    , public hde::interfaces::IHumanStateHorizon
    , public yarp::os::TypedReaderCallback<trintrin::msgs::HumanStateHorizon>
    , public yarp::os::PeriodicThread
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    HumanStateHorizon_nwc_yarp();
    ~HumanStateHorizon_nwc_yarp() override;

    // DeviceDriver interface
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    // PeriodicThread
    void run() override;
    void threadRelease() override;

    // TypedReaderCallback
    void onRead(trintrin::msgs::HumanStateHorizon& humanStateHorizon) override;

    // IHumanStateHorizon interface
    size_t getNumberOfSamples() const override;
    std::int32_t getTime(size_t index) const override;

    std::vector<std::string> getJointNames(size_t index) const override;
    std::string getBaseName(size_t index) const override;
    size_t getNumberOfJoints(size_t index) const override;

    std::vector<double> getJointPositions(size_t index) const override;
    std::vector<double> getJointVelocities(size_t index) const override;

    std::array<double, 3> getBasePosition(size_t index) const override;
    std::array<double, 4> getBaseOrientation(size_t index) const override;

    std::array<double, 6> getBaseVelocity(size_t index) const override;

    std::array<double, 3> getCoMPosition(size_t index) const override;
    std::array<double, 3> getCoMVelocity(size_t index) const override;
};

#endif // HDE_DEVICES_HUMANSTATEHORIZON_NWC_YARP
