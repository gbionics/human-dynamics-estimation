// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanWrench_nwc_yarp.h"

#include <trintrin/msgs/HumanWrench.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>

#include <ConnectionMonitor.h>

#include <iostream>
#include <mutex>

const std::string ClientName = "HumanWrench_nwc_yarp";
const std::string LogPrefix = ClientName + " :";
constexpr double LOG_PORT_DISCONNECTED_INTERVAL_S = 5.0;

using namespace hde::devices;

// ==============
// IMPL AND UTILS
// ==============

class HumanWrench_nwc_yarp::impl
{
public:
    std::mutex mtx;
    yarp::os::Network network;
    yarp::os::BufferedPort<trintrin::msgs::HumanWrench> inputPort;
    bool terminationCall = false;
    bool autoReconnect = false;
    std::string humanWrenchDataPortName;
    hde::ConnectionMonitor connectionMonitor;

    // Buffer HumanWrench variables
    std::vector<std::string> wrenchSourceNames;
    std::vector<double>  wrenches;
};

// ====================
// HUMANWRENCH CLIENT
// ====================

HumanWrench_nwc_yarp::HumanWrench_nwc_yarp()
    : PeriodicThread(1)
    , pImpl{new impl()}
{}

HumanWrench_nwc_yarp::~HumanWrench_nwc_yarp() = default;

// parsing the configuration file and connect ports
bool HumanWrench_nwc_yarp::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    // Data ports
    // TODO: where to check this port?
    if (!(config.check("humanWrenchDataPort") && config.find("humanWrenchDataPort").isString())) {
        yError() << LogPrefix << "humanWrenchData option does not exist or it is not a list";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    pImpl->humanWrenchDataPortName = config.find("humanWrenchDataPort").asString();
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
        yError() << LogPrefix << "Failed to open port" << pImpl->humanWrenchDataPortName;
        return false;
    }

    if (pImpl->autoReconnect) {
        pImpl->inputPort.setReporter(pImpl->connectionMonitor);
    }

    // ================
    // OPEN INPUT PORTS
    // ================
    yDebug() << LogPrefix << "Opening input ports";


    if (!yarp::os::Network::connect(pImpl->humanWrenchDataPortName,
                                    pImpl->inputPort.getName())) {
        yError() << LogPrefix << "Failed to connect " << pImpl->humanWrenchDataPortName
                 << " with " << pImpl->inputPort.getName();
        return false;
    }

    // We use callbacks on the input ports, the loop is a no-op
    start();

    yDebug() << LogPrefix << "Opened correctly";
    return true;
}

void HumanWrench_nwc_yarp::threadRelease()
{}

bool HumanWrench_nwc_yarp::close()
{
    pImpl->terminationCall = true;

    while(isRunning()) {
        stop();
    }

    return true;
}

void HumanWrench_nwc_yarp::run()
{
    if (pImpl->terminationCall) {
        return;
    }

    if (pImpl->autoReconnect && !pImpl->connectionMonitor.isConnected()) {
        yWarningThrottle(LOG_PORT_DISCONNECTED_INTERVAL_S)
            << LogPrefix << "Disconnected from" << pImpl->humanWrenchDataPortName
            << "- attempting to reconnect";
        yarp::os::Network::connect(pImpl->humanWrenchDataPortName,
                                   pImpl->inputPort.getName());
    }
}

// data are read from the port and saved in buffer variables
void HumanWrench_nwc_yarp::onRead(trintrin::msgs::HumanWrench& humanWrenchData)
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    if(!pImpl->terminationCall) {
        pImpl->wrenchSourceNames = humanWrenchData.wrenchSourceNames;

        pImpl->wrenches = humanWrenchData.wrenches;
    }
}

// method of IHumanWrench interface expose the buffer variables data
std::vector<std::string> HumanWrench_nwc_yarp::getWrenchSourceNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->wrenchSourceNames;
}

size_t HumanWrench_nwc_yarp::getNumberOfWrenchSources() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->wrenchSourceNames.size();
}

std::vector<double> HumanWrench_nwc_yarp::getWrenches() const
{
    std::lock_guard<std::mutex> lock(pImpl->mtx);
    return pImpl->wrenches;
}
