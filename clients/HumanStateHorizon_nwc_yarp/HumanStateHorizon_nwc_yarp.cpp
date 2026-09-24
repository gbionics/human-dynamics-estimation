// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanStateHorizon_nwc_yarp.h"

#include <trintrin/msgs/HumanStateHorizon.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <ConnectionMonitor.h>

#include <iostream>
#include <mutex>

const std::string ClientName = "HumanStateHorizon_nwc_yarp";
const std::string LogPrefix = ClientName + " :";
constexpr double LOG_PORT_DISCONNECTED_INTERVAL_S = 5.0;

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

// A single buffered human state sample, mirroring the fields of IHumanState plus its time.
struct BufferedSample
{
    std::int32_t time;

    std::vector<std::string> jointNames;
    std::string baseName;

    std::vector<double> jointPositions;
    std::vector<double> jointVelocities;

    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;

    std::array<double, 6> baseVelocity;

    std::array<double, 3> CoMPosition;
    std::array<double, 3> CoMVelocity;
};

class HumanStateHorizon_nwc_yarp::impl
{
public:
    std::mutex mtx;
    yarp::os::Network network;
    yarp::os::BufferedPort<trintrin::msgs::HumanStateHorizon> inputPort;
    bool terminationCall = false;
    bool autoReconnect = false;
    std::string humanStateHorizonDataPortName;
    hde::ConnectionMonitor connectionMonitor;

    // Buffer HumanStateHorizon variables
    std::vector<BufferedSample> horizon;
};

// ==========================
// IHUMANSTATEHORIZON CLIENT
// ==========================

HumanStateHorizon_nwc_yarp::HumanStateHorizon_nwc_yarp()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

HumanStateHorizon_nwc_yarp::~HumanStateHorizon_nwc_yarp() = default;

bool HumanStateHorizon_nwc_yarp::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    if (!(config.check("humanStateHorizonDataPort")
          && config.find("humanStateHorizonDataPort").isString())) {
        yError() << LogPrefix
                 << "humanStateHorizonDataPort option does not exist or it is not a string";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    pImpl->humanStateHorizonDataPortName =
        config.find("humanStateHorizonDataPort").asString();
    pImpl->autoReconnect = config.check("autoReconnect", yarp::os::Value(false)).asBool();

    // Initialize the network
    // TODO: is this required in every DeviceDriver?
    pImpl->network = yarp::os::Network();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        yError() << LogPrefix << "YARP server wasn't found active.";
        return false;
    }

    // ==========================
    // CONFIGURE INPUT DATA PORTS
    // ==========================
    yDebug() << LogPrefix << "Configuring input data ports";

    pImpl->inputPort.useCallback(*this);
    if (!pImpl->inputPort.open("...")) {
        yError() << LogPrefix << "Failed to open port" << pImpl->humanStateHorizonDataPortName;
        return false;
    }

    if (pImpl->autoReconnect) {
        pImpl->inputPort.setReporter(pImpl->connectionMonitor);
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << LogPrefix << "Opening input ports";

    if (!yarp::os::Network::connect(pImpl->humanStateHorizonDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << LogPrefix << "Failed to connect " << pImpl->humanStateHorizonDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << LogPrefix << "Opened correctly";
    return true;
}

void HumanStateHorizon_nwc_yarp::threadRelease()
{}

bool HumanStateHorizon_nwc_yarp::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void HumanStateHorizon_nwc_yarp::run()
{
    if (pImpl->terminationCall) {
        return;
    }

    if (pImpl->autoReconnect && !pImpl->connectionMonitor.isConnected()) {
        yWarningThrottle(LOG_PORT_DISCONNECTED_INTERVAL_S)
            << LogPrefix << "Disconnected from" << pImpl->humanStateHorizonDataPortName
            << "- attempting to reconnect";
        yarp::os::Network::connect(pImpl->humanStateHorizonDataPortName,
                                   pImpl->inputPort.getName());
    }
}

void HumanStateHorizon_nwc_yarp::onRead(trintrin::msgs::HumanStateHorizon& humanStateHorizonData)
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    if (pImpl->terminationCall) {
        return;
    }

    std::vector<BufferedSample> horizon;
    horizon.reserve(humanStateHorizonData.samples.size());

    for (const auto& sample : humanStateHorizonData.samples) {
        const auto& humanStateData = sample.state;

        BufferedSample bufferedSample;
        bufferedSample.time = sample.time;

        bufferedSample.jointNames = humanStateData.jointNames;
        bufferedSample.baseName = humanStateData.baseName;

        bufferedSample.jointPositions = humanStateData.positions;
        bufferedSample.jointVelocities = humanStateData.velocities;

        bufferedSample.basePosition = {humanStateData.baseOriginWRTGlobal.x,
                                       humanStateData.baseOriginWRTGlobal.y,
                                       humanStateData.baseOriginWRTGlobal.z};
        bufferedSample.baseOrientation = {humanStateData.baseOrientationWRTGlobal.w,
                                          humanStateData.baseOrientationWRTGlobal.imaginary.x,
                                          humanStateData.baseOrientationWRTGlobal.imaginary.y,
                                          humanStateData.baseOrientationWRTGlobal.imaginary.z};

        bufferedSample.baseVelocity = {humanStateData.baseVelocityWRTGlobal[0],
                                       humanStateData.baseVelocityWRTGlobal[1],
                                       humanStateData.baseVelocityWRTGlobal[2],
                                       humanStateData.baseVelocityWRTGlobal[3],
                                       humanStateData.baseVelocityWRTGlobal[4],
                                       humanStateData.baseVelocityWRTGlobal[5]};

        bufferedSample.CoMPosition = {humanStateData.CoMPositionWRTGlobal.x,
                                      humanStateData.CoMPositionWRTGlobal.y,
                                      humanStateData.CoMPositionWRTGlobal.z};
        bufferedSample.CoMVelocity = {humanStateData.CoMVelocityWRTGlobal.x,
                                      humanStateData.CoMVelocityWRTGlobal.y,
                                      humanStateData.CoMVelocityWRTGlobal.z};

        horizon.push_back(std::move(bufferedSample));
    }

    pImpl->horizon = std::move(horizon);
}

size_t HumanStateHorizon_nwc_yarp::getNumberOfSamples() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.size();
}

std::int32_t HumanStateHorizon_nwc_yarp::getTime(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).time;
}

std::vector<std::string> HumanStateHorizon_nwc_yarp::getJointNames(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).jointNames;
}

std::string HumanStateHorizon_nwc_yarp::getBaseName(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).baseName;
}

size_t HumanStateHorizon_nwc_yarp::getNumberOfJoints(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).jointPositions.size();
}

std::vector<double> HumanStateHorizon_nwc_yarp::getJointPositions(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).jointPositions;
}

std::vector<double> HumanStateHorizon_nwc_yarp::getJointVelocities(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).jointVelocities;
}

std::array<double, 3> HumanStateHorizon_nwc_yarp::getBasePosition(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).basePosition;
}

std::array<double, 4> HumanStateHorizon_nwc_yarp::getBaseOrientation(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).baseOrientation;
}

std::array<double, 6> HumanStateHorizon_nwc_yarp::getBaseVelocity(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).baseVelocity;
}

std::array<double, 3> HumanStateHorizon_nwc_yarp::getCoMPosition(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).CoMPosition;
}

std::array<double, 3> HumanStateHorizon_nwc_yarp::getCoMVelocity(size_t index) const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->horizon.at(index).CoMVelocity;
}
