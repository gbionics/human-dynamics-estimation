// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "RobotPositionController.h"
#include <hde/algorithms/MinJerkTrajGen.hpp>
#include <hde/interfaces/IHumanState.h>

#include <array>
#include <numeric>
#include <chrono>
#include <cmath>
#include <memory>

#include <thread>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

const std::string DeviceName = "RobotPositionController";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;

class RobotPositionController::impl
{
public:
    // Attached interface
    hde::interfaces::IHumanState* iHumanState = nullptr;

    // Polydriver device
    std::vector<yarp::dev::PolyDriver*> remoteControlBoards;
    yarp::dev::IEncoders* iEncoders = nullptr;
    yarp::dev::IControlMode* iControlMode = nullptr;
    yarp::dev::IPositionControl* iPosControl = nullptr;
    yarp::dev::IPositionDirect* iPosDirectControl = nullptr;
    yarp::os::Property options;

    double refSpeed;
    std::string controlMode;

    // Joint variables
    std::vector<int> nJointsVectorFromConfig;
    int totalControlBoardJoints;
    std::vector<double> jointPositionsVector;
    std::vector<double> encodersJointPositionsVector;
    std::vector<double> previousJointPositionsVector;
    std::vector<double> desiredJointPositionVector;

    // Min jerk trajectory
    double samplingTime;
    double smoothingTime;
    double initialSmoothingTime;
    int initialSmoothingCount;
    int maxSmoothingCount;
    std::vector<Eigen::VectorXd> posDirectRefJointPosVector;
    std::vector<std::unique_ptr<hde::algorithms::minJerkTrajGen>> minJerkTrajGeneratorVec;

    std::vector<std::string> jointNameListFromConfigControlBoards;
    std::vector<std::string> jointNameListFromHumanState;

    // Motion variable
    bool checkMotion = false;

    bool firstDataCheck = false;
    double jointPosVectorSum = 0; //Variabe to check first zero data
};

// =============================
// ROBOPOSITIONCONTROLLER DEVICE
// =============================

RobotPositionController::RobotPositionController()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

RobotPositionController::~RobotPositionController()
{}

bool RobotPositionController::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("refSpeed") && config.find("refSpeed").isFloat64())) {
        yError() << LogPrefix << "refSpeed option not found or not valid";
    }

    if (!(config.check("samplingTime") && config.find("samplingTime").isFloat64())) {
        yError() << LogPrefix << "samplingTime option not found or not valid";
    }

    if (!(config.check("smoothingTime") && config.find("smoothingTime").isFloat64())) {
        yError() << LogPrefix << "smoothingTime option not found or not valid";
    }

    if (!(config.check("controlMode") && config.find("controlMode").isString())) {
        yError() << LogPrefix << "controlMode option not found or not valid";
        return false;
    }

    if (!(config.check("controlBoardsList") && config.find("controlBoardsList").isList())) {
        yError() << LogPrefix << "controlBoardsList option not found or not valid";
        return false;
    }

    if (!(config.check("remotePrefix") && config.find("remotePrefix").isString())) {
        yError() << LogPrefix << "remotePrefix option not found or not valid";
        return false;
    }

    if (!(config.check("localPrefix") && config.find("localPrefix").isString())) {
        yError() << LogPrefix << "localPrefix option not found or not valid";
        return false;
    }

    // ===============================
    // PARSE THE CONFIGURATION OPTIONS
    // ===============================

    double period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();
    pImpl->controlMode = config.find("controlMode").asString();
    pImpl->refSpeed = config.find("refSpeed").asFloat64();
    pImpl->samplingTime = config.find("samplingTime").asFloat64();
    pImpl->smoothingTime = config.find("smoothingTime").asFloat64();
    pImpl->initialSmoothingCount = 0;
    yarp::os::Bottle* controlBoardsList = config.find("controlBoardsList").asList();
    const std::string remotePrefix  = config.find("remotePrefix").asString();
    const std::string localPrefix  = config.find("localPrefix").asString();

    if (pImpl->controlMode == "positionDirect")
    {
        if (!(config.check("initialSmoothingTime") && config.find("initialSmoothingTime").isFloat64())) {
            yInfo() << LogPrefix << "initialSmoothingTime option not found or not valid, using default value of 2.5 Seconds";
            pImpl->initialSmoothingTime = 2.5;
        }
        else {
            pImpl->initialSmoothingTime = config.find("initialSmoothingTime").asFloat64();
        }

        if (!(config.check("maxSmoothingCount") && config.find("maxSmoothingCount").isInt32())) {
            yInfo() << LogPrefix << "initialSmoothingCount option not found or not valid, using default value of 5000";
            pImpl->maxSmoothingCount = 5000;
        }
        else {
            pImpl->maxSmoothingCount = config.find("maxSmoothingCount").asInt32();
        }
    }

    yInfo() << LogPrefix << "*** ========================";
    yInfo() << LogPrefix << "*** Period                 :" << period;
    yInfo() << LogPrefix << "*** Control mode           :" << pImpl->controlMode;
    yInfo() << LogPrefix << "*** Reference speed        :" << pImpl->refSpeed;
    yInfo() << LogPrefix << "*** Sampling time          :" << pImpl->samplingTime;
    yInfo() << LogPrefix << "*** Smoothing time         :" << pImpl->smoothingTime;
    if (pImpl->controlMode == "positionDirect")
    {
        yInfo() << LogPrefix << "*** Initial Smoothing time :" << pImpl->initialSmoothingTime;
        yInfo() << LogPrefix << "*** Max smoothing count    :" << pImpl->maxSmoothingCount;
    }
    yInfo() << LogPrefix << "*** Control boards list    :" << controlBoardsList->toString();
    yInfo() << LogPrefix << "*** Remote prefix          :" << remotePrefix;
    yInfo() << LogPrefix << "*** Local prefix           :" << localPrefix;
    yInfo() << LogPrefix << "*** ========================";

    // ====================================
    // INITIALIZE THE REMOTE CONTROL BOARDS
    // ====================================

    std::vector<std::string> controlBoards;

    for (unsigned index = 0; index < controlBoardsList->size(); index++) {
        controlBoards.push_back(controlBoardsList->get(index).asString());
    }

    // Set the size of remote control boards vector
    pImpl->remoteControlBoards.resize(controlBoards.size());
    pImpl->nJointsVectorFromConfig.resize(controlBoards.size());
    pImpl->minJerkTrajGeneratorVec.resize(controlBoards.size());
    pImpl->posDirectRefJointPosVector.resize(controlBoards.size());

    // Open the control boards
    size_t boardCount = 0;
    int remoteControlBoardJoints = 0;
    for (const auto& controlBoard : controlBoards) {


        // Get joint names of the control board from configuration
        if (!(config.check(controlBoard) && config.find(controlBoard).isList())) {
            yError() << LogPrefix << "joint list option not found or not valid for " << controlBoard << " control board";
            return false;
        }

        yarp::os::Bottle* jointsList = config.find(controlBoard).asList();
        yarp::os::Bottle axesNames;
        yarp::os::Bottle & axesList = axesNames.addList();

        // Set the number of joints from config file
        pImpl->nJointsVectorFromConfig.at(boardCount) = jointsList->size();

        for (unsigned index = 0; index < jointsList->size(); index++) {
            pImpl->jointNameListFromConfigControlBoards.push_back(jointsList->get(index).asString());
            axesList.addString(jointsList->get(index).asString());
        }


        pImpl->options.put("device", "remotecontrolboardremapper");
        pImpl->options.put("axesNames",axesNames.get(0));

        yarp::os::Bottle remoteControlBoards;
        yarp::os::Bottle & remoteControlBoardsList = remoteControlBoards.addList();
        remoteControlBoardsList.addString( remotePrefix + "/" + controlBoard);

        pImpl->options.put("remoteControlBoards",remoteControlBoards.get(0));
        pImpl->options.put("localPortPrefix", localPrefix);

        yarp::os::Property & remoteControlBoardsOpts = pImpl->options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteControlBoardsOpts.put("writeStrict","on");


        pImpl->remoteControlBoards.at(boardCount) = new yarp::dev::PolyDriver;
        pImpl->remoteControlBoards.at(boardCount)->open(pImpl->options);

        if (!pImpl->remoteControlBoards.at(boardCount)->isValid()) {
            yError() << LogPrefix << "Failed to open the remote control board device";
            return false;
        }

        // Get control mode interface
        if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iControlMode) || !pImpl->iControlMode) {
            yError() << LogPrefix << "Failed to view the IControlMode interface from the " << controlBoard << " remote control board device";
            return false;
        }

        // Get encoder interface
        if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iEncoders) || !pImpl->iEncoders) {
            yError() << LogPrefix << "Failed to view the IEncoder interface from the " << controlBoard << " remote control board device";
            return false;
        }

        // Get joint axes from encoder interface
        pImpl->iEncoders->getAxes(&remoteControlBoardJoints);

        if (pImpl->controlMode == "position") {

            // Check position control interface
            if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iPosControl) || !pImpl->iPosControl) {
                yError() << LogPrefix << "Failed to view the IPositionControl interface from the " << controlBoard << " remote control board device";
                return false;
            }

            // Set control mode
            for (unsigned i = 0; i < remoteControlBoardJoints; i++) {
                pImpl->iControlMode->setControlMode(i,VOCAB_CM_POSITION);
                pImpl->iPosControl->setRefSpeed(i, pImpl->refSpeed);
            }

        }

        if (pImpl->controlMode == "positionDirect") {

            // Check position control direct interface
            if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iPosDirectControl) || !pImpl->iPosDirectControl) {
                yError() << LogPrefix << "Failed to view the IPositionDirectControl interface from the " << controlBoard << " remote control board device";
                return false;
            }

            // Set control mode
            for (unsigned i = 0; i < remoteControlBoardJoints; i++) {
                pImpl->iControlMode->setControlMode(i,VOCAB_CM_POSITION_DIRECT);
            }

            // Get initial joint positions from encoders
            std::vector<double> initEncoderJointPositions;
            initEncoderJointPositions.resize(remoteControlBoardJoints);

            bool getEncodersOk = false;
            int MAX_COUNT_GET_ENCODERS = 5000;
            int countgetEncoders = 0;

            while(!getEncodersOk)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                getEncodersOk = pImpl->iEncoders->getEncoders(initEncoderJointPositions.data());
                countgetEncoders++;
                if(countgetEncoders>MAX_COUNT_GET_ENCODERS)
                {
                    yError()<< LogPrefix << "Cannot read encoder for "<< controlBoard;
                    return false;
                }
            }

            // Initialize min jerk object pointer
            pImpl->minJerkTrajGeneratorVec.at(boardCount) =
                std::make_unique<hde::algorithms::minJerkTrajGen>(
                    remoteControlBoardJoints, pImpl->samplingTime, pImpl->initialSmoothingTime);
            pImpl->posDirectRefJointPosVector.at(boardCount).resize(remoteControlBoardJoints);

            // Set min jerk object initial values
            pImpl->minJerkTrajGeneratorVec.at(boardCount)->init(
                Eigen::Map<const Eigen::VectorXd>(initEncoderJointPositions.data(), remoteControlBoardJoints));
        }


        pImpl->options.clear();
        boardCount++;
    }

    // Compute total joints from control boards
    pImpl->totalControlBoardJoints = std::accumulate(pImpl->nJointsVectorFromConfig.begin(), pImpl->nJointsVectorFromConfig.end(), 0);

    // resize and initialize vectors
    pImpl->desiredJointPositionVector.resize(pImpl->jointNameListFromConfigControlBoards.size());

    return true;
}

bool RobotPositionController::close()
{
    for(auto& board : pImpl->remoteControlBoards) {
        board->close();
    }

    return true;
}

void RobotPositionController::run()
{
    // Get joint positions from iHumanState interface
    pImpl->jointPositionsVector = pImpl->iHumanState->getJointPositions();
    pImpl->jointNameListFromHumanState = pImpl->iHumanState->getJointNames();

    // Check for first data
    pImpl->jointPosVectorSum = std::accumulate(pImpl->jointPositionsVector.begin(), pImpl->jointPositionsVector.end(), 0.0);

    // TODO: This is not the best way to check
    // This check is to see if the first data read from the  IHumanState interface is all zero angles
    if (pImpl->jointPosVectorSum != 0 && !pImpl->firstDataCheck) {
        pImpl->firstDataCheck = true;

        // Set the previous joint positions vector first time
        pImpl->previousJointPositionsVector = pImpl->iHumanState->getJointPositions();
    }

    if (pImpl->firstDataCheck) {

        // Set the joint position values array for iPositionControl interface
        for (unsigned controlBoardJointIndex = 0; controlBoardJointIndex < pImpl->jointNameListFromConfigControlBoards.size(); controlBoardJointIndex++) {
            for (unsigned humanStateJointIndex = 0; humanStateJointIndex < pImpl->jointNameListFromHumanState.size(); humanStateJointIndex++) {
                if (pImpl->jointNameListFromConfigControlBoards.at(controlBoardJointIndex) == pImpl->jointNameListFromHumanState.at(humanStateJointIndex)) {
                    if (std::fabs(pImpl->jointPositionsVector.at(humanStateJointIndex) - pImpl->previousJointPositionsVector.at(humanStateJointIndex)) < 10) {
                        pImpl->desiredJointPositionVector.at(controlBoardJointIndex) = pImpl->jointPositionsVector.at(humanStateJointIndex)*(180/M_PI);
                    }
                    else {
                        pImpl->desiredJointPositionVector.at(controlBoardJointIndex) =  pImpl->previousJointPositionsVector.at(humanStateJointIndex)*(180/M_PI);
                    }
                     break;
                }
            }
        }

        // Set the desired joint positions and ask to move
        int jointNumber = 0;
        for (size_t boardCount = 0; boardCount < pImpl->remoteControlBoards.size(); boardCount++) {

            pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iEncoders);

            // Get joints from iEncoder interface
            int joints;
            pImpl->iEncoders->getAxes(&joints);

            // Read joint position through IEncoder interface
            pImpl->encodersJointPositionsVector.resize(joints);
            pImpl->iEncoders->getEncoders(pImpl->encodersJointPositionsVector.data());

            if (pImpl->controlMode == "position") {
                pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iPosControl);
            }

            if (pImpl->controlMode == "positionDirect") {
                pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iPosDirectControl);
            }

            for (int j = 0; j < joints; j++) {

                if (pImpl->controlMode == "position") {
                    if (j < pImpl->nJointsVectorFromConfig.at(boardCount)) {
                        pImpl->iPosControl->positionMove(j, pImpl->desiredJointPositionVector.at(jointNumber));
                        jointNumber++;
                    }
                }

                if (pImpl->controlMode == "positionDirect") {
                    if (j < pImpl->nJointsVectorFromConfig.at(boardCount))  {
                        pImpl->posDirectRefJointPosVector.at(boardCount)[j] = pImpl->desiredJointPositionVector.at(jointNumber);
                        jointNumber++;
                    }
                    else {
                        pImpl->posDirectRefJointPosVector.at(boardCount)[j] = pImpl->encodersJointPositionsVector.at(j);
                    }
                }
            }

            if (pImpl->controlMode == "positionDirect") {

                // Call min jerk trajecotry to smooth reference positions

                // Update initial smoothing counter
                if (pImpl->initialSmoothingCount < pImpl->maxSmoothingCount)
                {
                    pImpl->initialSmoothingCount++;
                    pImpl->minJerkTrajGeneratorVec.at(boardCount)->setT(pImpl->initialSmoothingTime);

                }
                else
                {
                    pImpl->minJerkTrajGeneratorVec.at(boardCount)->setT(pImpl->smoothingTime);
                }

                pImpl->minJerkTrajGeneratorVec.at(boardCount)->computeNextValues(
                    pImpl->posDirectRefJointPosVector.at(boardCount));
                pImpl->iPosDirectControl->setPositions(
                    pImpl->minJerkTrajGeneratorVec.at(boardCount)->getPos().data());
            }
        }

        if (pImpl->controlMode == "position") {
            while(!pImpl->checkMotion) {
                pImpl->iPosControl->checkMotionDone(&pImpl->checkMotion);
            }

            pImpl->checkMotion = false;
        }
    }
}

bool RobotPositionController::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (!pImpl->iHumanState && !poly->view(pImpl->iHumanState)) {
        yError() << LogPrefix << "Failed to view the IHumanState interface from the PolyDriver";
        return false;
    }

    // ===================
    // CHECK THE INTERFACE
    // ===================

    while (pImpl->iHumanState->getNumberOfJoints() == 0
        || pImpl->iHumanState->getNumberOfJoints() != pImpl->iHumanState->getJointNames().size()) {
        yInfo() << LogPrefix << "IHumanState interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
    }

    // Check the joint numbers match
    /*if (pImpl->iHumanState->getNumberOfJoints() != pImpl->totalControlBoardJoints) {
     yError() << "Number of joints mismatch between the control boards and IHumanState interface";
     return false;
     }*/

    yDebug() << LogPrefix << "Read" << pImpl->iHumanState->getNumberOfJoints() << "joints";

    // ====
    // MISC
    // ====

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop";
        return false;
    }

    yInfo() << LogPrefix << "attach() successful";
    return true;
}

void RobotPositionController::threadRelease()
{}

bool RobotPositionController::detach()
{
    if (pImpl->controlMode == "positionDirect") {

        // Set the position control mode
        for (size_t boardCount = 0; boardCount < pImpl->remoteControlBoards.size(); boardCount++) {

            // Get encoder interface
            if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iEncoders) || !pImpl->iEncoders) {
                yError() << LogPrefix << "Failed to view the IEncoder interface from the (" << boardCount << ") remote control board device";
                return false;
            }

            // Get joint axes from encoder interface
            int remoteControlBoardJoints;
            pImpl->iEncoders->getAxes(&remoteControlBoardJoints);

            // Get control mode interface
            if (!pImpl->remoteControlBoards.at(boardCount)->view(pImpl->iControlMode) || !pImpl->iControlMode) {
                yError() << LogPrefix << "Failed to view the IControlMode interface from the (" << boardCount << ") remote control board device";
                return false;
            }

            // Set control mode
            for (unsigned joint = 0; joint < remoteControlBoardJoints; joint++) {
                pImpl->iControlMode->setControlMode(joint,VOCAB_CM_POSITION);
            }

        }
    }

    while (isRunning()) {
        yarp::os::PeriodicThread::stop();
    }

    pImpl->iHumanState       = nullptr;
    pImpl->iEncoders         = nullptr;
    pImpl->iControlMode      = nullptr;
    pImpl->iPosControl       = nullptr;
    pImpl->iPosDirectControl = nullptr;

    return true;
}
