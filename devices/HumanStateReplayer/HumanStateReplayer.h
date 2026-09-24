// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_DEVICES_HUMANSTATEREPLAYER_H
#define HDE_DEVICES_HUMANSTATEREPLAYER_H

#include <hde/interfaces/IHumanState.h>
#include <thrift/HumanStateReplayerService.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/os/PeriodicThread.h>

#include <memory>

namespace hde::devices {

class HumanStateReplayer final
    : public yarp::dev::DeviceDriver
    , public yarp::os::PeriodicThread
    , public hde::interfaces::IHumanState
    , public hde::msgs::HumanStateReplayerService
{
public:
    HumanStateReplayer();
    ~HumanStateReplayer() override;

    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    void run() override;
    void threadRelease() override;

    std::vector<std::string> getJointNames() const override;
    std::string getBaseName() const override;
    size_t getNumberOfJoints() const override;
    std::vector<double> getJointPositions() const override;
    std::vector<double> getJointVelocities() const override;
    std::array<double, 3> getBasePosition() const override;
    std::array<double, 4> getBaseOrientation() const override;
    std::array<double, 6> getBaseVelocity() const override;
    std::array<double, 3> getCoMPosition() const override;
    std::array<double, 3> getCoMVelocity() const override;

    bool start() override;
    bool pause() override;
    bool reset() override;

private:
    struct impl;
    std::unique_ptr<impl> pImpl;
};

} // namespace hde::devices

#endif // HDE_DEVICES_HUMANSTATEREPLAYER_H
