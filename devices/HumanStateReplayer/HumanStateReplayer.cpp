// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanStateReplayer.h"

#include <trintrin/msgs/HumanState.h>

#include <matioCpp/matioCpp.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/RpcServer.h>

#include <array>
#include <mutex>
#include <numeric>
#include <vector>

namespace
{
    YARP_LOG_COMPONENT(HUMANSTATEREPLAYER, "hde.devices.HumanStateReplayer")
}
using namespace hde::devices;

enum class ReplayState
{
    IDLE,
    PLAYING,
    PAUSED
};

struct HumanStateReplayer::impl
{
    mutable std::mutex mutex;

    yarp::os::BufferedPort<trintrin::msgs::HumanState> outputPort;
    yarp::os::RpcServer rpcPort;

    ReplayState state{ReplayState::IDLE};
    size_t currentFrame{0};
    bool loop{false};

    std::string baseName;
    std::vector<std::string> jointNames;
    size_t numFrames{0};
    double inferredPeriod{0.01};

    std::vector<std::array<double, 3>> basePositions;
    std::vector<std::array<double, 4>> baseOrientations;
    std::vector<std::array<double, 6>> baseVelocities;
    std::vector<std::vector<double>> jointPositions;
    std::vector<std::vector<double>> jointVelocities;

    bool loadMatFile(const std::string& matFilePath);
};

// ==================
// MAT file loading
// ==================

static bool readFixedDimChannel(const matioCpp::Variable& channelStruct,
                                size_t nDims,
                                size_t nFrames,
                                std::vector<std::vector<double>>& out)
{
    auto data = channelStruct["data"].asMultiDimensionalArray<double>();
    if (!data.isValid()) {
        return false;
    }
    out.resize(nFrames, std::vector<double>(nDims));
    const double* raw = data.data();
    for (size_t t = 0; t < nFrames; ++t) {
        for (size_t d = 0; d < nDims; ++d) {
            out[t][d] = raw[d + nDims * t];
        }
    }
    return true;
}

bool HumanStateReplayer::impl::loadMatFile(const std::string& matFilePath)
{
    matioCpp::File file(matFilePath, matioCpp::FileMode::ReadOnly);
    if (!file.isOpen()) {
        yCError(HUMANSTATEREPLAYER) << "Failed to open MAT file:" << matFilePath;
        return false;
    }

    auto varNames = file.variableNames();
    if (varNames.empty()) {
        yCError(HUMANSTATEREPLAYER) << "No variables found in MAT file";
        return false;
    }

    matioCpp::Variable topVar = file.read(varNames[0]);
    if (!topVar.isValid()) {
        yCError(HUMANSTATEREPLAYER) << "Failed to read top-level variable:" << varNames[0];
        return false;
    }

    // Read timestamps from base_position to determine numFrames
    auto tsVar = topVar["human_state"]["base_position"]["timestamps"];
    if (!tsVar.isValid()) {
        yCError(HUMANSTATEREPLAYER) << "Missing human_state::base_position::timestamps";
        return false;
    }
    auto timestamps = tsVar.asVector<double>();
    numFrames = timestamps.size();
    if (numFrames == 0) {
        yCError(HUMANSTATEREPLAYER) << "Dataset contains no frames";
        return false;
    }

    // Infer playback period from mean inter-frame interval
    if (numFrames > 1) {
        double totalTime = timestamps[numFrames - 1] - timestamps[0];
        inferredPeriod = totalTime / static_cast<double>(numFrames - 1);
    }

    // Read joint names from elements_names CellArray
    auto elemNamesVar = topVar["joints_state"]["positions"]["elements_names"];
    if (!elemNamesVar.isValid()) {
        yCError(HUMANSTATEREPLAYER) << "Missing joints_state::positions::elements_names";
        return false;
    }
    auto elemNamesCellArray = elemNamesVar.asCellArray();
    size_t nJoints = elemNamesCellArray.numberOfElements();
    jointNames.resize(nJoints);
    for (size_t i = 0; i < nJoints; ++i) {
        jointNames[i] = elemNamesCellArray[i].asVector<char>()();
    }

    // base_position [3 x numFrames]
    {
        std::vector<std::vector<double>> tmp;
        if (!readFixedDimChannel(topVar["human_state"]["base_position"], 3, numFrames, tmp)) {
            yCError(HUMANSTATEREPLAYER) << "Failed to read base_position";
            return false;
        }
        basePositions.resize(numFrames);
        for (size_t t = 0; t < numFrames; ++t) {
            basePositions[t] = {tmp[t][0], tmp[t][1], tmp[t][2]};
        }
    }

    // base_orientation [4 x numFrames]
    {
        std::vector<std::vector<double>> tmp;
        if (!readFixedDimChannel(topVar["human_state"]["base_orientation"], 4, numFrames, tmp)) {
            yCError(HUMANSTATEREPLAYER) << "Failed to read base_orientation";
            return false;
        }
        baseOrientations.resize(numFrames);
        for (size_t t = 0; t < numFrames; ++t) {
            baseOrientations[t] = {tmp[t][0], tmp[t][1], tmp[t][2], tmp[t][3]};
        }
    }

    // base_velocity [6 x numFrames]
    {
        std::vector<std::vector<double>> tmp;
        if (!readFixedDimChannel(topVar["human_state"]["base_velocity"], 6, numFrames, tmp)) {
            yCError(HUMANSTATEREPLAYER) << "Failed to read base_velocity";
            return false;
        }
        baseVelocities.resize(numFrames);
        for (size_t t = 0; t < numFrames; ++t) {
            baseVelocities[t] = {tmp[t][0], tmp[t][1], tmp[t][2], tmp[t][3], tmp[t][4], tmp[t][5]};
        }
    }

    // joints_state::positions [nJoints x numFrames]
    if (!readFixedDimChannel(topVar["joints_state"]["positions"], nJoints, numFrames, jointPositions)) {
        yCError(HUMANSTATEREPLAYER) << "Failed to read joints_state::positions";
        return false;
    }

    // joints_state::velocities [nJoints x numFrames]
    if (!readFixedDimChannel(topVar["joints_state"]["velocities"], nJoints, numFrames, jointVelocities)) {
        yCError(HUMANSTATEREPLAYER) << "Failed to read joints_state::velocities";
        return false;
    }

    return true;
}

// ====================
// HumanStateReplayer
// ====================

HumanStateReplayer::HumanStateReplayer()
    : PeriodicThread(0.01)
    , pImpl{new impl()}
{}

HumanStateReplayer::~HumanStateReplayer()
{
    close();
}

bool HumanStateReplayer::open(yarp::os::Searchable& config)
{
    if (!config.check("matFile") || !config.find("matFile").isString()) {
        yCError(HUMANSTATEREPLAYER) << "matFile option not found or not valid";
        return false;
    }
    if (!config.check("outputPort") || !config.find("outputPort").isString()) {
        yCError(HUMANSTATEREPLAYER) << "outputPort option not found or not valid";
        return false;
    }

    const std::string matFilePath = config.find("matFile").asString();
    const std::string outputPortName = config.find("outputPort").asString();
    const std::string rpcPortName = outputPortName + "/rpc:i";
    pImpl->baseName = config.check("baseName", yarp::os::Value("Pelvis")).asString();
    pImpl->loop = config.check("loop", yarp::os::Value(false)).asBool();
    const double speedMultiplier = config.check("speedMultiplier", yarp::os::Value(1.0)).asFloat64();

    if (!pImpl->loadMatFile(matFilePath)) {
        return false;
    }

    // Determine thread period: explicit config overrides auto-inferred value
    if (config.check("period") && config.find("period").isFloat64()) {
        setPeriod(config.find("period").asFloat64() / speedMultiplier);
    }
    else {
        setPeriod(pImpl->inferredPeriod / speedMultiplier);
        yCInfo(HUMANSTATEREPLAYER) << "Inferred period:" << pImpl->inferredPeriod / speedMultiplier << "s";
    }

    if (!pImpl->outputPort.open(outputPortName)) {
        yCError(HUMANSTATEREPLAYER) << "Failed to open output port" << outputPortName;
        return false;
    }

    if (!pImpl->rpcPort.open(rpcPortName)) {
        yCError(HUMANSTATEREPLAYER) << "Failed to open RPC port" << rpcPortName;
        return false;
    }
    this->yarp().attachAsServer(pImpl->rpcPort);

    yCInfo(HUMANSTATEREPLAYER) << "Loaded" << pImpl->numFrames << "frames,"
                               << pImpl->jointNames.size() << "joints";

    if (!PeriodicThread::start()) {
        yCError(HUMANSTATEREPLAYER) << "Failed to start periodic thread";
        return false;
    }

    return true;
}

bool HumanStateReplayer::close()
{
    if (PeriodicThread::isRunning()) {
        PeriodicThread::stop();
    }
    pImpl->outputPort.close();
    pImpl->rpcPort.close();
    return true;
}

void HumanStateReplayer::run()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);

    if (pImpl->state != ReplayState::PLAYING) {
        return;
    }

    trintrin::msgs::HumanState& msg = pImpl->outputPort.prepare();

    const auto& bp = pImpl->basePositions[pImpl->currentFrame];
    const auto& bo = pImpl->baseOrientations[pImpl->currentFrame];
    const auto& bv = pImpl->baseVelocities[pImpl->currentFrame];
    const auto& jp = pImpl->jointPositions[pImpl->currentFrame];
    const auto& jv = pImpl->jointVelocities[pImpl->currentFrame];

    msg.baseOriginWRTGlobal = {bp[0], bp[1], bp[2]};
    msg.baseOrientationWRTGlobal = {bo[0], {bo[1], bo[2], bo[3]}};
    msg.baseVelocityWRTGlobal.resize(6);
    for (size_t i = 0; i < 6; ++i) {
        msg.baseVelocityWRTGlobal[i] = bv[i];
    }

    const size_t nJoints = pImpl->jointNames.size();
    msg.jointNames.resize(nJoints);
    msg.positions.resize(nJoints);
    msg.velocities.resize(nJoints);
    for (size_t i = 0; i < nJoints; ++i) {
        msg.jointNames[i] = pImpl->jointNames[i];
        msg.positions[i] = jp[i];
        msg.velocities[i] = jv[i];
    }

    // CoM not logged by HumanLogger; publish zeros
    msg.CoMPositionWRTGlobal = {0.0, 0.0, 0.0};
    msg.CoMVelocityWRTGlobal = {0.0, 0.0, 0.0};
    msg.baseName = pImpl->baseName;

    pImpl->outputPort.write(true);

    if (++pImpl->currentFrame >= pImpl->numFrames) {
        if (pImpl->loop) {
            pImpl->currentFrame = 0;
        }
        else {
            pImpl->currentFrame = pImpl->numFrames - 1;
            pImpl->state = ReplayState::IDLE;
        }
    }
}

void HumanStateReplayer::threadRelease() {}

// ====================
// IHumanState interface
// ====================

std::vector<std::string> HumanStateReplayer::getJointNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->jointNames;
}

std::string HumanStateReplayer::getBaseName() const
{
    return pImpl->baseName;
}

size_t HumanStateReplayer::getNumberOfJoints() const
{
    return pImpl->jointNames.size();
}

std::vector<double> HumanStateReplayer::getJointPositions() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->jointPositions.empty()) {
        return {};
    }
    return pImpl->jointPositions[pImpl->currentFrame];
}

std::vector<double> HumanStateReplayer::getJointVelocities() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->jointVelocities.empty()) {
        return {};
    }
    return pImpl->jointVelocities[pImpl->currentFrame];
}

std::array<double, 3> HumanStateReplayer::getBasePosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->basePositions.empty()) {
        return {};
    }
    return pImpl->basePositions[pImpl->currentFrame];
}

std::array<double, 4> HumanStateReplayer::getBaseOrientation() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->baseOrientations.empty()) {
        return {};
    }
    return pImpl->baseOrientations[pImpl->currentFrame];
}

std::array<double, 6> HumanStateReplayer::getBaseVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->baseVelocities.empty()) {
        return {};
    }
    return pImpl->baseVelocities[pImpl->currentFrame];
}

std::array<double, 3> HumanStateReplayer::getCoMPosition() const
{
    return {0.0, 0.0, 0.0};
}

std::array<double, 3> HumanStateReplayer::getCoMVelocity() const
{
    return {0.0, 0.0, 0.0};
}

// ====================
// RPC service
// ====================

bool HumanStateReplayer::start()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->state == ReplayState::IDLE || pImpl->state == ReplayState::PAUSED) {
        pImpl->state = ReplayState::PLAYING;
        yCInfo(HUMANSTATEREPLAYER) << "Started playback at frame" << pImpl->currentFrame;
        return true;
    }
    return false;
}

bool HumanStateReplayer::pause()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    if (pImpl->state == ReplayState::PLAYING) {
        pImpl->state = ReplayState::PAUSED;
        yCInfo(HUMANSTATEREPLAYER) << "Paused at frame" << pImpl->currentFrame;
        return true;
    }
    return false;
}

bool HumanStateReplayer::reset()
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    pImpl->state = ReplayState::IDLE;
    pImpl->currentFrame = 0;
    yCInfo(HUMANSTATEREPLAYER) << "Reset to frame 0";
    return true;
}
