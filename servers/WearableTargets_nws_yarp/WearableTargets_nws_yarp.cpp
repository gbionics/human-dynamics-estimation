// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "WearableTargets_nws_yarp.h"
#include <hde/interfaces/IWearableTargets.h>
#include <trintrin/msgs/WearableTargets.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>

#include <unordered_map>

const std::string DeviceName = "WearableTargets_nws_yarp";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::servers;

// =======
// Helpers
// =======

const std::map<hde::KinematicTargetType, trintrin::msgs::KinematicTargetType> mapKinematicTargetType = {
    {hde::KinematicTargetType::none, trintrin::msgs::KinematicTargetType::NONE},
    {hde::KinematicTargetType::pose, trintrin::msgs::KinematicTargetType::POSE},
    {hde::KinematicTargetType::poseAndVelocity, trintrin::msgs::KinematicTargetType::POSEANDVELOCITY},
    {hde::KinematicTargetType::position, trintrin::msgs::KinematicTargetType::POSITION},
    {hde::KinematicTargetType::positionAndVelocity, trintrin::msgs::KinematicTargetType::POSITIONANDVELOCITY},
    {hde::KinematicTargetType::orientation, trintrin::msgs::KinematicTargetType::ORIENTATION},
    {hde::KinematicTargetType::orientationAndVelocity, trintrin::msgs::KinematicTargetType::ORIENTATIONANDVELOCITY},
    {hde::KinematicTargetType::gravity, trintrin::msgs::KinematicTargetType::GRAVITY},
    {hde::KinematicTargetType::floorContact, trintrin::msgs::KinematicTargetType::FLOORCONTACT},
};

trintrin::msgs::KinematicTargetType generateMsgKinematicTargetType(const hde::KinematicTargetType kinematicTargetType)
{
    return mapKinematicTargetType.at(kinematicTargetType);
}

trintrin::msgs::VectorXYZ generateMsgVectorXYZ(const iDynTree::Vector3& input)
{
    return {input[0], input[1], input[2]};
}

trintrin::msgs::Quaternion generateMsgQuaternion(const iDynTree::Rotation& input)
{
    iDynTree::Vector4 quaternion;
    input.getQuaternion(quaternion);
    return {quaternion[0], {quaternion[1], quaternion[2], quaternion[3]}};
}

trintrin::msgs::Transform generateMsgTransform(const iDynTree::Transform& input)
{
    return {generateMsgVectorXYZ(input.getPosition()), generateMsgQuaternion(input.getRotation())};
}

class WearableTargets_nws_yarp::impl
{
public:
    hde::interfaces::IWearableTargets* iWearableTargets = nullptr;
    yarp::os::BufferedPort<trintrin::msgs::WearableTargets> outputPort;

    bool firstRun = true;

    // buffer variables
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::WearableSensorTarget>> wearableTargets;
};

WearableTargets_nws_yarp::WearableTargets_nws_yarp()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

WearableTargets_nws_yarp::~WearableTargets_nws_yarp() {}

bool WearableTargets_nws_yarp::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("outputPort") && config.find("outputPort").isString())) {
        yError() << LogPrefix << "outputPort option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    std::string outputPortName = config.find("outputPort").asString();

    // =============
    // OPEN THE PORT
    // =============

    if (!pImpl->outputPort.open(outputPortName)) {
        yError() << LogPrefix << "Failed to open port" << outputPortName;
        return false;
    }

    // ================
    // SETUP THE THREAD
    // ================

    setPeriod(period);

    return true;
}


bool WearableTargets_nws_yarp::close()
{
    pImpl->outputPort.close();
    return true;
}

void WearableTargets_nws_yarp::run()
{
    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        std::vector<hde::TargetName> targetsName = pImpl->iWearableTargets->getAllTargetsName();

        for (auto targetName : targetsName )
        {
            pImpl->wearableTargets[targetName] = pImpl->iWearableTargets->getTarget(targetName);
        }
    }

    trintrin::msgs::WearableTargets& data = pImpl->outputPort.prepare();

    for (const auto& wearableTargetEntry : pImpl->wearableTargets) {
        data.targets[wearableTargetEntry.first] = {wearableTargetEntry.second.get()->wearableName,
                                                   wearableTargetEntry.second.get()->modelLinkName,
                                                   generateMsgKinematicTargetType(wearableTargetEntry.second.get()->targetType),
                                                   generateMsgVectorXYZ(wearableTargetEntry.second.get()->position),
                                                   generateMsgQuaternion(wearableTargetEntry.second.get()->rotation),
                                                   generateMsgVectorXYZ(wearableTargetEntry.second.get()->linearVelocity),
                                                   generateMsgVectorXYZ(wearableTargetEntry.second.get()->angularVelocity),
                                                   generateMsgTransform(wearableTargetEntry.second.get()->calibrationWorldToMeasurementWorld),
                                                   generateMsgTransform(wearableTargetEntry.second.get()->calibrationMeasurementToLink),
                                                   generateMsgVectorXYZ(wearableTargetEntry.second.get()->positionScaleFactor),
                                                   wearableTargetEntry.second.get()->contactActive,
                                                   wearableTargetEntry.second.get()->contactThreshold,
                                                   generateMsgVectorXYZ(wearableTargetEntry.second.get()->contactForce)};
                                                   
    }

    // Stream the data though the port
    pImpl->outputPort.write(true);
}

bool WearableTargets_nws_yarp::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (pImpl->iWearableTargets || !poly->view(pImpl->iWearableTargets) || !pImpl->iWearableTargets) {
        yError() << LogPrefix << "Failed to view the IWearableTargets interface from the PolyDriver";
        return false;
    }

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop";
        return false;
    }

    yDebug() << LogPrefix << "attach() successful";

    return true;
}

void WearableTargets_nws_yarp::threadRelease() {}

bool WearableTargets_nws_yarp::detach()
{
    while (isRunning()) {
        stop();
    }

    while (!pImpl->outputPort.isClosed()) {
        pImpl->outputPort.close();
    }

    pImpl->iWearableTargets = nullptr;

    return true;
}

bool WearableTargets_nws_yarp::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This server accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    return attach(driver->poly);
}

bool WearableTargets_nws_yarp::detachAll()
{
    return detach();
}
