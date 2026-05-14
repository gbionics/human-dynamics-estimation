// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include "HumanStateProvider.h"
#include "IKWorkerPool.h"

#include <hde/algorithms/DynamicalInverseKinematics.hpp>
#include <hde/algorithms/InverseVelocityKinematics.hpp>
#include <hde/utils/iDynTreeUtils.hpp>

#include <Wearable/IWear/IWear.h>
#include <iDynTree/InverseKinematics.h>
#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Vocab.h>

#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Traversal.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <numeric>
#include <stack>
#include <string>
#include <thread>
#include <unordered_map>

/*!
 * @brief analyze model and list of segments to create all possible segment pairs
 *
 * @param[in] model the full model
 * @param[in] humanSegments list of segments on which look for the possible pairs
 * @param[out] framePairs resulting list of all possible pairs. First element is parent, second is
 * child
 * @param[out] framePairIndeces indeces in the humanSegments list of the pairs in framePairs
 */
static void createEndEffectorsPairs(
    const iDynTree::Model& model,
    std::vector<SegmentInfo>& humanSegments,
    std::vector<std::pair<std::string, std::string>>& framePairs,
    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>>& framePairIndeces);

static bool getReducedModel(const iDynTree::Model& modelInput,
                            const std::string& parentFrame,
                            const std::string& endEffectorFrame,
                            iDynTree::Model& modelOutput);

const std::string DeviceName = "HumanStateProvider";
const std::string LogPrefix = DeviceName + " :";
constexpr double DefaultPeriod = 0.01;

using namespace hde::devices;
using namespace wearable;

// ==============
// IMPL AND UTILS
// ==============

using ModelJointName = std::string;

using InverseVelocityKinematicsSolverName = std::string;

using FloatingBaseName = std::string;

// Struct that contains all the data exposed by the HumanState interface
struct SolutionIK
{
    std::vector<double> jointPositions;
    std::vector<double> jointVelocities;

    std::array<double, 3> basePosition;
    std::array<double, 4> baseOrientation;

    std::array<double, 6> baseVelocity;

    std::array<double, 3> CoMPosition;
    std::array<double, 3> CoMVelocity;

    void clear()
    {
        jointPositions.clear();
        jointVelocities.clear();
    }
};

enum SolverIK
{
    global,
    pairwised,
    dynamical
};

enum rpcCommand
{
    empty,
    calibrateAll,
    calibrateAllWithWorld,
    calibrateAllWorldYaw,
    calibrateRelativeLink,
    setRotationOffset,
    resetCalibration,
    resetAll
};

static std::unordered_map<std::string, hde::KinematicTargetType> const stringToKinemaitcTargetType =
    {{"pose", hde::KinematicTargetType::pose},
     {"poseAndVelocity", hde::KinematicTargetType::poseAndVelocity},
     {"position", hde::KinematicTargetType::position},
     {"positionAndVelocity", hde::KinematicTargetType::positionAndVelocity},
     {"orientation", hde::KinematicTargetType::orientation},
     {"orientationAndVelocity", hde::KinematicTargetType::orientationAndVelocity},
     {"gravity", hde::KinematicTargetType::gravity},
     {"floorContact", hde::KinematicTargetType::floorContact}};

// Container of data coming from the wearable interface
struct WearableStorage
{
    // Maps [model joint / link name] ==> [wearable virtual sensor name]
    //
    // E.g. [Pelvis] ==> [XsensSuit::vLink::Pelvis]. Read from the configuration.
    //
    std::unordered_map<hde::ModelLinkName, WearableName> modelToWearable_LinkName;

    // Maps [wearable virtual sensor name] ==> [virtual sensor]
    std::unordered_map<WearableName, SensorPtr<const sensor::IVirtualLinkKinSensor>> linkSensorsMap;
};

class HumanStateProvider::impl
{
public:
    // Attached interface
    wearable::IWear* iWear = nullptr;

    bool allowIKFailures;

    double period;
    mutable std::mutex mutex;

    bool resetIntegrator = false;
    std::mutex mutexFlagIntegrator;

    // Rpc
    class CmdParser;
    std::unique_ptr<CmdParser> commandPro;
    yarp::os::RpcServer rpcPort;
    bool applyRpcCommand();

    // Wearable variables
    WearableStorage wearableStorage;

    // Model variables
    iDynTree::Model humanModel;
    FloatingBaseName floatingBaseFrame;

    std::vector<std::string> jointList;

    std::vector<SegmentInfo> segments;
    std::vector<LinkPairInfo> linkPairs;

    // Target
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::WearableSensorTarget>> wearableTargets;

    // Buffers
    std::unordered_map<std::string, iDynTree::Transform> linkTransformMatrices;
    std::unordered_map<std::string, iDynTree::Transform> linkTransformMatricesRaw;
    std::unordered_map<std::string, iDynTree::Twist> linkVelocities;
    iDynTree::VectorDynSize jointConfigurationSolution;
    iDynTree::VectorDynSize jointCalibrationSolution;
    iDynTree::VectorDynSize jointVelocitiesSolution;
    iDynTree::Transform baseTransformSolution;
    iDynTree::Twist baseVelocitySolution;

    std::unordered_map<std::string, hde::utils::idyntree::rotation::RotationDistance>
        linkErrorOrientations;
    std::unordered_map<std::string, iDynTree::Vector3> linkErrorAngularVelocities;

    // IK parameters
    int ikPoolSize{1};
    int maxIterationsIK;
    double costTolerance;
    std::string linearSolverName;
    yarp::os::Value ikPoolOption;
    std::unique_ptr<IKWorkerPool> ikPool;
    SolutionIK solution;
    InverseVelocityKinematicsSolverName inverseVelocityKinematicsSolver;

    double posTargetWeight;
    double rotTargetWeight;
    double linVelTargetWeight;
    double angVelTargetWeight;
    double costRegularization;

    double dynamicalIKMeasuredLinearVelocityGain;
    double dynamicalIKMeasuredAngularVelocityGain;
    double dynamicalIKLinearCorrectionGain;
    double dynamicalIKAngularCorrectionGain;
    double dynamicalIKJointVelocityLimit;

    std::vector<std::string> custom_jointsVelocityLimitsNames;
    std::vector<iDynTree::JointIndex> custom_jointsVelocityLimitsIndexes;
    iDynTree::VectorDynSize custom_jointsVelocityLimitsValues;
    // Custom Constraint Form: lowerBound<=A*X<=upperBuond
    iDynTree::MatrixDynSize
        customConstraintMatrix; // A, CxN matrix; C: number of Constraints, N: number of
                                // system states: Dofs+6 in floating-based robot
    std::vector<std::string> customConstraintVariables; // X, Nx1  Vector : variables names
    std::vector<iDynTree::JointIndex>
        customConstraintVariablesIndex; // X, Nx1  Vector : variables index
    iDynTree::VectorDynSize customConstraintUpperBound; // upperBuond, Cx1 Vector
    iDynTree::VectorDynSize customConstraintLowerBound; // lowerBound, Cx1 Vector
    iDynTree::VectorDynSize baseVelocityUpperLimit;
    iDynTree::VectorDynSize baseVelocityLowerLimit;
    double k_u, k_l;

    // Secondary calibration
    void ereaseTargetCalibration(const hde::TargetName& targetName);
    void ereaseTargetsCalibration();
    void selectChainJointsAndLinksForSecondaryCalibration(
        const std::string& linkName,
        const std::string& childLinkName,
        std::vector<iDynTree::JointIndex>& jointZeroIndices,
        std::vector<iDynTree::LinkIndex>& linkToCalibrateIndices);
    void computeSecondaryCalibrationRotationsForChain(
        const std::vector<iDynTree::JointIndex>& jointZeroIndices,
        const iDynTree::Transform& refLinkForCalibrationTransform,
        const std::vector<iDynTree::LinkIndex>& linkToCalibrateIndices,
        const hde::TargetName& refLinkForCalibrationName);

    SolverIK ikSolver;

    // flags
    bool useDirectBaseMeasurement;
    bool useFixedBase;

    iDynTree::InverseKinematics globalIK;
    hde::algorithms::InverseVelocityKinematics
        inverseVelocityKinematics; // used for computing joint velocity in global IK solution
    hde::algorithms::DynamicalInverseKinematics dynamicalInverseKinematics;

    // clock
    double lastTime{-1.0};

    // kinDynComputation
    std::unique_ptr<iDynTree::KinDynComputations> kinDynComputations;
    iDynTree::Vector3 worldGravity;

    // get input data
    bool updateWearableTargets();

    // solver initialization and update
    bool createLinkPairs();
    bool initializePairwisedInverseKinematicsSolver();
    bool initializeGlobalInverseKinematicsSolver();
    bool initializeDynamicalInverseKinematicsSolver();
    bool solvePairwisedInverseKinematicsSolver();
    bool solveGlobalInverseKinematicsSolver();
    bool solveDynamicalInverseKinematics();

    // optimization targets
    bool updateInverseKinematicTargets();
    bool addInverseKinematicTargets();

    // dynamical inverse kinematic targets
    bool addDynamicalInverseKinematicsTargets();

    bool computeLinksOrientationErrors(
        std::unordered_map<std::string, iDynTree::Transform> linkDesiredOrientations,
        iDynTree::VectorDynSize jointConfigurations,
        iDynTree::Transform floatingBasePose,
        std::unordered_map<std::string, hde::utils::idyntree::rotation::RotationDistance>&
            linkErrorOrientations);
    bool computeLinksAngularVelocityErrors(
        std::unordered_map<std::string, iDynTree::Twist> linkDesiredVelocities,
        iDynTree::VectorDynSize jointConfigurations,
        iDynTree::Transform floatingBasePose,
        iDynTree::VectorDynSize jointVelocities,
        iDynTree::Twist baseVelocity,
        std::unordered_map<std::string, iDynTree::Vector3>& linkAngularVelocityError);

    // constructor
    impl();

    // attach IWear interface
    bool attach(yarp::dev::PolyDriver* poly);
};

// ===============
// RPC PORT PARSER
// ===============

class HumanStateProvider::impl::CmdParser : public yarp::os::PortReader
{

public:
    std::atomic<rpcCommand> cmdStatus{rpcCommand::empty};
    std::string parentLinkName;
    std::string childLinkName;
    std::string refLinkName;
    // variables for manual calibration
    std::atomic<double> roll; // [deg]
    std::atomic<double> pitch; // [deg]
    std::atomic<double> yaw; // [deg]

    void resetInternalVariables()
    {
        parentLinkName = "";
        childLinkName = "";
        cmdStatus = rpcCommand::empty;
    }

    bool read(yarp::os::ConnectionReader& connection) override
    {
        yarp::os::Bottle command, response;
        if (command.read(connection)) {
            if (command.get(0).asString() == "help") {
                response.addVocab32(yarp::os::Vocab32::encode("many"));
                response.addString("The following commands can be used to apply a secondary "
                                   "calibration assuming the subject is in the zero configuration "
                                   "of the model for the calibrated links. \n");
                response.addString("Enter <calibrateAll> to apply a secondary calibration for all "
                                   "the targets using the measured base pose \n");
                response.addString(
                    "Enter <calibrateAllWithWorld <refTarget>> to apply a secondary calibration "
                    "for all the targets assuming the <refTarget> to be in the world origin \n");
                response.addString(
                    "Enter <calibrateAllWorldYaw> to remove the yaw offset for all the data \n");
                response.addString("Enter <setRotationOffset <targetName> <r p y [deg]>> to apply "
                                   "a secondary calibration for the given target using the given "
                                   "rotation offset (defined using rpy)\n");
                response.addString(
                    "Enter <calibrateRelativeLink <parentTargetName> <childTargetName>> to apply a "
                    "secondary calibration for the child target using the parent target "
                    "measurement as reference \n");
                response.addString("Enter <reset <targetName>> to remove secondary calibration for "
                                   "the given target \n");
                response.addString("Enter <resetAll> to remove all the secondary calibrations");
            }
            else if (command.get(0).asString() == "calibrateRelativeLink"
                     && !command.get(1).isNull() && !command.get(2).isNull()) {
                this->parentLinkName = command.get(1).asString();
                this->childLinkName = command.get(2).asString();
                response.addString(
                    "Entered command <calibrateRelativeLink> is correct, trying to set offset of "
                    + this->childLinkName + " using " + this->parentLinkName + " as reference");
                this->cmdStatus = rpcCommand::calibrateRelativeLink;
            }
            else if (command.get(0).asString() == "calibrateAllWorldYaw") {
                this->parentLinkName = "";
                response.addString("Entered command <calibrateAllWorldYaw> is correct, trying to "
                                   "set yaw calibration for all the targets");
                this->cmdStatus = rpcCommand::calibrateAllWorldYaw;
            }
            else if (command.get(0).asString() == "calibrateAll") {
                this->parentLinkName = "";
                response.addString("Entered command <calibrateAll> is correct, trying to set "
                                   "offset calibration for all the targets");
                this->cmdStatus = rpcCommand::calibrateAll;
            }
            else if (command.get(0).asString() == "calibrateAllWithWorld") {
                this->parentLinkName = "";
                this->refLinkName = command.get(1).asString();
                response.addString("Entered command <calibrateAllWithWorld> is correct, trying to "
                                   "set offset calibration for all the targets, and setting target "
                                   + this->refLinkName + " to the origin");
                this->cmdStatus = rpcCommand::calibrateAllWithWorld;
            }
            else if (command.get(0).asString() == "setRotationOffset" && !command.get(1).isNull()
                     && command.get(2).isFloat64() && command.get(3).isFloat64()
                     && command.get(4).isFloat64()) {
                this->parentLinkName = command.get(1).asString();
                this->roll = command.get(2).asFloat64();
                this->pitch = command.get(3).asFloat64();
                this->yaw = command.get(4).asFloat64();
                response.addString("Entered command <calibrate> is correct, trying to set rotation "
                                   "offset for the target "
                                   + this->parentLinkName);
                this->cmdStatus = rpcCommand::setRotationOffset;
            }
            else if (command.get(0).asString() == "resetAll") {
                response.addString("Entered command <resetAll> is correct,  trying to remove "
                                   "calibration transforms (right and left) for all the targets");
                this->cmdStatus = rpcCommand::resetAll;
            }
            else if (command.get(0).asString() == "reset" && !command.get(1).isNull()) {
                this->parentLinkName = command.get(1).asString();
                response.addString("Entered command <reset> is correct, trying to remove "
                                   "calibration transforms (right and left) for the target "
                                   + this->parentLinkName);
                this->cmdStatus = rpcCommand::resetCalibration;
            }
            else {
                response.addString(
                    "Entered command is incorrect. Enter help to know available commands");
            }
        }
        else {
            resetInternalVariables();
            return false;
        }

        yarp::os::ConnectionWriter* reply = connection.getWriter();

        if (reply != NULL) {
            response.write(*reply);
        }
        else
            return false;

        return true;
    }
};

// ===========
// CONSTRUCTOR
// ===========

HumanStateProvider::impl::impl()
    : commandPro(new CmdParser())
{}

// =========================
// HUMANSTATEPROVIDER DEVICE
// =========================

HumanStateProvider::HumanStateProvider()
    : PeriodicThread(DefaultPeriod)
    , pImpl{new impl()}
{}

HumanStateProvider::~HumanStateProvider() {}

bool HumanStateProvider::open(yarp::os::Searchable& config)
{
    // ===============================
    // CHECK THE CONFIGURATION OPTIONS
    // ===============================

    if (!(config.check("period") && config.find("period").isFloat64())) {
        yInfo() << LogPrefix << "Using default period:" << DefaultPeriod << "s";
    }

    if (!(config.check("urdf") && config.find("urdf").isString())) {
        yError() << LogPrefix << "urdf option not found or not valid";
        return false;
    }

    if (!(config.check("ikSolver") && config.find("ikSolver").isString())) {
        yError() << LogPrefix << "ikSolver option not found or not valid";
        return false;
    }

    if (!(config.check("jointList") && config.find("jointList").isList())) {
        yInfo() << LogPrefix
                << "jointList option not found or not valid, all the model joints are selected.";
        pImpl->jointList.clear();
    }
    else {
        auto jointListBottle = config.find("jointList").asList();
        for (size_t it = 0; it < jointListBottle->size(); it++) {
            pImpl->jointList.push_back(jointListBottle->get(it).asString());
        }
    }

    std::vector<double> calibrationJointConfiguration;
    if (!config.check("calibrationJointConfiguration")) {
        yInfo()
            << LogPrefix
            << "calibrationJointConfiguration option not found, using zero configuration instead.";
        calibrationJointConfiguration.clear();
    }
    else {
        auto calibrationJointConfigurationValue = config.find("calibrationJointConfiguration");
        if (!calibrationJointConfigurationValue.isList()) {
            yError() << LogPrefix << "Param calibrationJointConfiguration must be a list";
            return false;
        }
        auto calibrationJointConfigurationBottle = calibrationJointConfigurationValue.asList();
        if (calibrationJointConfigurationBottle->size() != pImpl->jointList.size()) {
            yError() << LogPrefix
                     << "Parameter calibrationJointConfiguration must have the same size of the "
                        "list of selected joints!";
            return false;
        }
        for (size_t it = 0; it < calibrationJointConfigurationBottle->size(); it++) {
            calibrationJointConfiguration.push_back(
                calibrationJointConfigurationBottle->get(it).asFloat64());
        }
    }

    std::string baseFrameName;
    if (config.check("floatingBaseFrame") && config.find("floatingBaseFrame").isList()) {
        baseFrameName = config.find("floatingBaseFrame").asList()->get(0).asString();
        pImpl->useFixedBase = false;
        yWarning() << LogPrefix
                   << "'floatingBaseFrame' configuration option as list is deprecated. Please use "
                      "a string with the model base name only.";
    }
    else if (config.check("floatingBaseFrame") && config.find("floatingBaseFrame").isString()) {
        baseFrameName = config.find("floatingBaseFrame").asString();
        pImpl->useFixedBase = false;
    }
    else if (config.check("fixedBaseFrame") && config.find("fixedBaseFrame").isList()) {
        baseFrameName = config.find("fixedBaseFrame").asList()->get(0).asString();
        pImpl->useFixedBase = true;
        yWarning() << LogPrefix
                   << "'fixedBaseFrame' configuration option as list is deprecated. Please use a "
                      "string with the model base name only.";
    }
    else if (config.check("fixedBaseFrame") && config.find("fixedBaseFrame").isString()) {
        baseFrameName = config.find("fixedBaseFrame").asString();
        pImpl->useFixedBase = true;
    }
    else {
        yError() << LogPrefix << "BaseFrame option not found or not valid";
        return false;
    }

    yarp::os::Bottle& linksGroup = config.findGroup("WEARABLE_SENSOR_TARGETS");
    if (linksGroup.isNull()) {
        yError() << LogPrefix << "Failed to find group WEARABLE_SENSOR_TARGETS";
        return false;
    }
    for (size_t i = 1; i < linksGroup.size(); ++i) {
        if (!(linksGroup.get(i).isList() && linksGroup.get(i).asList()->size() == 2)) {
            yError() << LogPrefix
                     << "Childs of WEARABLE_SENSOR_TARGETS must be lists of two elements";
            return false;
        }
        yarp::os::Bottle* list = linksGroup.get(i).asList();
        std::string key = list->get(0).asString();
        yarp::os::Bottle* listContent = list->get(1).asList();

        if (!((listContent->size() == 3) && (listContent->get(0).isString())
              && (listContent->get(1).isString()))) {
            yError() << LogPrefix << "Link list must have two strings";
            return false;
        }
    }

    yarp::os::Bottle& fixedRightTransformGroup = config.findGroup("MEASUREMENT_TO_LINK_TRANSFORMS");
    if (!fixedRightTransformGroup.isNull()) {
        for (size_t i = 1; i < fixedRightTransformGroup.size(); ++i) {
            if (!(fixedRightTransformGroup.get(i).isList()
                  && fixedRightTransformGroup.get(i).asList()->size() == 2)) {
                yError() << LogPrefix
                         << "Childs of MEASUREMENT_TO_LINK_TRANSFORMS must be lists of 2 elements";
                return false;
            }
            yarp::os::Bottle* list = fixedRightTransformGroup.get(i).asList();
            std::string linkName = list->get(0).asString();
            yarp::os::Bottle* listContent = list->get(1).asList();

            // check if MEASUREMENT_TO_LINK_TRANSFORMS matrix is passed (16 elements)
            if (!((listContent->size() == 16) && (listContent->get(0).isFloat64())
                  && (listContent->get(1).isFloat64()) && (listContent->get(2).isFloat64())
                  && (listContent->get(3).isFloat64()) && (listContent->get(4).isFloat64())
                  && (listContent->get(5).isFloat64()) && (listContent->get(6).isFloat64())
                  && (listContent->get(7).isFloat64()) && (listContent->get(8).isFloat64())
                  && (listContent->get(9).isFloat64()) && (listContent->get(10).isFloat64())
                  && (listContent->get(11).isFloat64()) && (listContent->get(12).isFloat64())
                  && (listContent->get(13).isFloat64()) && (listContent->get(14).isFloat64())
                  && (listContent->get(15).isFloat64()))) {
                yError() << LogPrefix << "MEASUREMENT_TO_LINK_TRANSFORMS " << linkName
                         << " must have 16 double values describing the rotation matrix";
                return false;
            }

            yInfo() << LogPrefix << "MEASUREMENT_TO_LINK_TRANSFORMS added for target " << linkName;
        }
    }

    yarp::os::Bottle& fixedLeftTransformGroup = config.findGroup("WORLD_TO_MEASUREMENT_TRANSFORMS");
    if (!fixedLeftTransformGroup.isNull()) {
        for (size_t i = 1; i < fixedLeftTransformGroup.size(); ++i) {
            if (!(fixedLeftTransformGroup.get(i).isList()
                  && fixedLeftTransformGroup.get(i).asList()->size() == 2)) {
                yError() << LogPrefix
                         << "Childs of WORLD_TO_MEASUREMENT_TRANSFORMS must be lists of 2 elements";
                return false;
            }
            yarp::os::Bottle* list = fixedLeftTransformGroup.get(i).asList();
            std::string linkName = list->get(0).asString();
            yarp::os::Bottle* listContent = list->get(1).asList();

            // check if WORLD_TO_MEASUREMENT_TRANSFORMS matrix is passed (9 elements)
            if (!((listContent->size() == 16) && (listContent->get(0).isFloat64())
                  && (listContent->get(1).isFloat64()) && (listContent->get(2).isFloat64())
                  && (listContent->get(3).isFloat64()) && (listContent->get(4).isFloat64())
                  && (listContent->get(5).isFloat64()) && (listContent->get(6).isFloat64())
                  && (listContent->get(7).isFloat64()) && (listContent->get(8).isFloat64())
                  && (listContent->get(9).isFloat64()) && (listContent->get(10).isFloat64())
                  && (listContent->get(11).isFloat64()) && (listContent->get(12).isFloat64())
                  && (listContent->get(13).isFloat64()) && (listContent->get(14).isFloat64())
                  && (listContent->get(15).isFloat64()))) {
                yError() << LogPrefix << "WORLD_TO_MEASUREMENT_TRANSFORMS " << linkName
                         << " must have 16 double values describing the rotation matrix";
                return false;
            }

            yInfo() << LogPrefix << "WORLD_TO_MEASUREMENT_TRANSFORMS added for target " << linkName;
        }
    }

    yarp::os::Bottle& positionScaleFactorGroup =
        config.findGroup("MEASUREMENT_POSITION_SCALE_FACTOR");
    if (!positionScaleFactorGroup.isNull()) {
        for (size_t i = 1; i < positionScaleFactorGroup.size(); ++i) {
            if (!(positionScaleFactorGroup.get(i).isList()
                  && positionScaleFactorGroup.get(i).asList()->size() == 2)) {
                yError()
                    << LogPrefix
                    << "Childs of MEASUREMENT_POSITION_SCALE_FACTOR must be lists of 2 elements";
                return false;
            }
            yarp::os::Bottle* list = positionScaleFactorGroup.get(i).asList();
            std::string linkName = list->get(0).asString();

            if (list->get(1).isList()) {
                yarp::os::Bottle* listContent = list->get(1).asList();

                // check if FIXED_SENSOR_ROTATION matrix is passed (9 elements)
                if (!((listContent->size() == 3) && (listContent->get(0).isFloat64())
                      && (listContent->get(1).isFloat64()) && (listContent->get(2).isFloat64()))) {
                    yError() << LogPrefix << "MEASUREMENT_POSITION_SCALE_FACTOR " << linkName
                             << " must have 3 double values describing the scaling factor for x, "
                                "y, and z axis";
                    return false;
                }
                yInfo() << LogPrefix << "MEASUREMENT_POSITION_SCALE_FACTOR added for target "
                        << linkName;
            }
        }
    }

    // Check if scale_factor_all options are found
    bool xScaleFactorAllFlag = positionScaleFactorGroup.check("x_scale_factor_all");
    bool yScaleFactorAllFlag = positionScaleFactorGroup.check("y_scale_factor_all");
    bool zScaleFactorAllFlag = positionScaleFactorGroup.check("z_scale_factor_all");

    if (xScaleFactorAllFlag && !positionScaleFactorGroup.find("x_scale_factor_all").isFloat64()) {
        yError() << LogPrefix << "x_scale_factor_all must be a double ";
        return false;
    }
    if (yScaleFactorAllFlag && !positionScaleFactorGroup.find("y_scale_factor_all").isFloat64()) {
        yError() << LogPrefix << "y_scale_factor_all must be a double ";
        return false;
    }
    if (zScaleFactorAllFlag && !positionScaleFactorGroup.find("z_scale_factor_all").isFloat64()) {
        yError() << LogPrefix << "z_scale_factor_all must be a double ";
        return false;
    }

    yarp::os::Bottle& contactThresholdGroup = config.findGroup("CONTACT_THRESHOLDS");
    if (!contactThresholdGroup.isNull()) {
        for (size_t i = 1; i < contactThresholdGroup.size(); ++i) {
            if (!(contactThresholdGroup.get(i).isList()
                  && contactThresholdGroup.get(i).asList()->size() == 2)) {
                yError() << LogPrefix << "Childs of CONTACT_THRESHOLDS must be lists of 2 elements";
                return false;
            }
            yarp::os::Bottle* list = contactThresholdGroup.get(i).asList();
            std::string targetName = list->get(0).asString();

            if (!list->find(targetName).isFloat64()) {
                yError() << LogPrefix << "CONTACT_THRESHOLDS " << targetName << " must be a float";
                return false;
            }

            yInfo() << LogPrefix << "CONTACT_THRESHOLDS added for target " << targetName;
        }
    }

    // =======================================
    // PARSE THE GENERAL CONFIGURATION OPTIONS
    // =======================================

    std::string solverName = config.find("ikSolver").asString();
    if (solverName == "global")
        pImpl->ikSolver = SolverIK::global;
    else if (solverName == "pairwised")
        pImpl->ikSolver = SolverIK::pairwised;
    else if (solverName == "dynamical")
        pImpl->ikSolver = SolverIK::dynamical;
    else {
        yError() << LogPrefix << "ikSolver " << solverName << " not found";
        return false;
    }

    const std::string urdfFileName = config.find("urdf").asString();
    pImpl->floatingBaseFrame = baseFrameName;
    pImpl->period = config.check("period", yarp::os::Value(DefaultPeriod)).asFloat64();

    setPeriod(pImpl->period);

    for (size_t i = 1; i < linksGroup.size(); ++i) {
        yarp::os::Bottle* listContent = linksGroup.get(i).asList()->get(1).asList();

        ModelLinkName modelLinkName = listContent->get(0).asString();
        WearableName wearableName = listContent->get(1).asString();
        std::string targetTypeString = listContent->get(2).asString();
        auto it = stringToKinemaitcTargetType.find(targetTypeString);
        if (it == stringToKinemaitcTargetType.end()) {
            yError() << LogPrefix << "Target Type not valid [" << targetTypeString << "]";
            return false;
        }
        hde::KinematicTargetType targetType = it->second;
        hde::TargetName targetName = linksGroup.get(i).asList()->get(0).asString();

        if (pImpl->wearableTargets.find(targetName) != pImpl->wearableTargets.end()) {
            yError() << LogPrefix << "Duplicated target name found [" << targetName << "]";
            return false;
        }

        pImpl->wearableTargets[targetName] =
            std::make_shared<hde::WearableSensorTarget>(wearableName, modelLinkName, targetType);
    }

    for (size_t i = 1; i < fixedLeftTransformGroup.size(); ++i) {
        hde::TargetName targetName = fixedLeftTransformGroup.get(i).asList()->get(0).asString();
        yarp::os::Bottle* fixedLeftRotationMatrixValues =
            fixedLeftTransformGroup.get(i).asList()->get(1).asList();

        iDynTree::Rotation fixedLeftRotation =
            iDynTree::Rotation(fixedLeftRotationMatrixValues->get(0).asFloat64(),
                               fixedLeftRotationMatrixValues->get(1).asFloat64(),
                               fixedLeftRotationMatrixValues->get(2).asFloat64(),
                               fixedLeftRotationMatrixValues->get(4).asFloat64(),
                               fixedLeftRotationMatrixValues->get(5).asFloat64(),
                               fixedLeftRotationMatrixValues->get(6).asFloat64(),
                               fixedLeftRotationMatrixValues->get(8).asFloat64(),
                               fixedLeftRotationMatrixValues->get(9).asFloat64(),
                               fixedLeftRotationMatrixValues->get(10).asFloat64());
        iDynTree::Position fixedLeftPositionOffset =
            iDynTree::Position(fixedLeftRotationMatrixValues->get(3).asFloat64(),
                               fixedLeftRotationMatrixValues->get(7).asFloat64(),
                               fixedLeftRotationMatrixValues->get(11).asFloat64());

        if (pImpl->wearableTargets.find(targetName) == pImpl->wearableTargets.end()) {
            yError() << LogPrefix << "Left calibration rotation for not existing target ["
                     << targetName << "]";
            return false;
        }

        pImpl->wearableTargets[targetName].get()->calibrationWorldToMeasurementWorld.setRotation(
            fixedLeftRotation);
        pImpl->wearableTargets[targetName].get()->calibrationWorldToMeasurementWorld.setPosition(
            fixedLeftPositionOffset);
        yInfo() << LogPrefix << "Adding Fixed Transform for " << targetName << "==>"
                << pImpl->wearableTargets[targetName]
                       .get()
                       ->calibrationWorldToMeasurementWorld.toString();
    }

    for (size_t i = 1; i < fixedRightTransformGroup.size(); ++i) {
        hde::TargetName targetName = fixedRightTransformGroup.get(i).asList()->get(0).asString();
        yarp::os::Bottle* fixedRightRotationMatrixValues =
            fixedRightTransformGroup.get(i).asList()->get(1).asList();

        iDynTree::Rotation fixedRightRotation =
            iDynTree::Rotation(fixedRightRotationMatrixValues->get(0).asFloat64(),
                               fixedRightRotationMatrixValues->get(1).asFloat64(),
                               fixedRightRotationMatrixValues->get(2).asFloat64(),
                               fixedRightRotationMatrixValues->get(4).asFloat64(),
                               fixedRightRotationMatrixValues->get(5).asFloat64(),
                               fixedRightRotationMatrixValues->get(6).asFloat64(),
                               fixedRightRotationMatrixValues->get(8).asFloat64(),
                               fixedRightRotationMatrixValues->get(9).asFloat64(),
                               fixedRightRotationMatrixValues->get(10).asFloat64());

        iDynTree::Position fixedRightPositionOffset =
            iDynTree::Position(fixedRightRotationMatrixValues->get(3).asFloat64(),
                               fixedRightRotationMatrixValues->get(7).asFloat64(),
                               fixedRightRotationMatrixValues->get(11).asFloat64());
        if (pImpl->wearableTargets.find(targetName) == pImpl->wearableTargets.end()) {
            yError() << LogPrefix << "Right calibration rotation for not existing target ["
                     << targetName << "]";
            return false;
        }

        pImpl->wearableTargets[targetName].get()->calibrationMeasurementToLink.setRotation(
            fixedRightRotation);
        pImpl->wearableTargets[targetName].get()->calibrationMeasurementToLink.setPosition(
            fixedRightPositionOffset);
        pImpl->wearableTargets[targetName].get()->calibrationMeasurementToLinkInitial =
            pImpl->wearableTargets[targetName].get()->calibrationMeasurementToLink;
        yInfo()
            << LogPrefix << "Adding Fixed Rotation for " << targetName << "==>"
            << pImpl->wearableTargets[targetName].get()->calibrationMeasurementToLink.toString();
    }

    for (size_t i = 1; i < positionScaleFactorGroup.size(); ++i) {
        hde::TargetName targetName = positionScaleFactorGroup.get(i).asList()->get(0).asString();
        if (targetName == "x_scale_factor_all" || targetName == "y_scale_factor_all"
            || targetName == "z_scale_factor_all")
            continue;
        yarp::os::Bottle* positionScaleFactorValues =
            positionScaleFactorGroup.get(i).asList()->get(1).asList();

        iDynTree::Vector3 positionScaleFactor;
        if (xScaleFactorAllFlag)
            positionScaleFactor.setVal(
                0, positionScaleFactorGroup.find("x_scale_factor_all").asFloat64());
        else
            positionScaleFactor.setVal(0, positionScaleFactorValues->get(0).asFloat64());

        if (yScaleFactorAllFlag)
            positionScaleFactor.setVal(
                1, positionScaleFactorGroup.find("y_scale_factor_all").asFloat64());
        else
            positionScaleFactor.setVal(1, positionScaleFactorValues->get(1).asFloat64());

        if (zScaleFactorAllFlag)
            positionScaleFactor.setVal(
                2, positionScaleFactorGroup.find("z_scale_factor_all").asFloat64());
        else
            positionScaleFactor.setVal(2, positionScaleFactorValues->get(2).asFloat64());

        if (pImpl->wearableTargets.find(targetName) == pImpl->wearableTargets.end()) {
            yError() << LogPrefix << "Position Scale Factor for not existing target [" << targetName
                     << "]";
            return false;
        }

        pImpl->wearableTargets[targetName].get()->positionScaleFactor = positionScaleFactor;
        yInfo() << LogPrefix << "Adding Scale factor for " << targetName << "==>"
                << positionScaleFactor.toString();
    }

    for (size_t i = 1; i < contactThresholdGroup.size(); ++i) {
        hde::TargetName targetName = contactThresholdGroup.get(i).asList()->get(0).asString();
        double contactThreshold =
            contactThresholdGroup.get(i).asList()->find(targetName).asFloat64();

        if (pImpl->wearableTargets.find(targetName) == pImpl->wearableTargets.end()) {
            yError() << LogPrefix << "Contact threshold for not existing target [" << targetName
                     << "]";
            return false;
        }

        pImpl->wearableTargets[targetName].get()->contactThreshold = contactThreshold;
        yInfo() << LogPrefix << "Adding contact threshold for " << targetName << "==>"
                << contactThreshold;
    }

    // ==========================================
    // PARSE THE DEPENDENDT CONFIGURATION OPTIONS
    // ==========================================

    if (pImpl->ikSolver == SolverIK::pairwised || pImpl->ikSolver == SolverIK::global) {
        if (!(config.check("allowIKFailures") && config.find("allowIKFailures").isBool())) {
            yError() << LogPrefix << "allowFailures option not found or not valid";
            return false;
        }
        if (!(config.check("maxIterationsIK") && config.find("maxIterationsIK").isInt32())) {
            yError() << LogPrefix << "maxIterationsIK option not found or not valid";
            return false;
        }

        if (!(config.check("costTolerance") && config.find("costTolerance").isFloat64())) {
            yError() << LogPrefix << "costTolerance option not found or not valid";
            return false;
        }
        if (!(config.check("ikLinearSolver") && config.find("ikLinearSolver").isString())) {
            yError() << LogPrefix << "ikLinearSolver option not found or not valid";
            return false;
        }
        if (!(config.check("posTargetWeight") && config.find("posTargetWeight").isFloat64())) {
            yError() << LogPrefix << "posTargetWeight option not found or not valid";
            return false;
        }

        if (!(config.check("rotTargetWeight") && config.find("rotTargetWeight").isFloat64())) {
            yError() << LogPrefix << "rotTargetWeight option not found or not valid";
            return false;
        }
        if (!(config.check("costRegularization")
              && config.find("costRegularization").isFloat64())) {
            yError() << LogPrefix << "costRegularization option not found or not valid";
            return false;
        }

        pImpl->allowIKFailures = config.find("allowIKFailures").asBool();
        pImpl->maxIterationsIK = config.find("maxIterationsIK").asInt32();
        pImpl->costTolerance = config.find("costTolerance").asFloat64();
        pImpl->linearSolverName = config.find("ikLinearSolver").asString();
        pImpl->posTargetWeight = config.find("posTargetWeight").asFloat64();
        pImpl->rotTargetWeight = config.find("rotTargetWeight").asFloat64();
        pImpl->costRegularization = config.find("costRegularization").asFloat64();
    }

    if (pImpl->ikSolver == SolverIK::global || pImpl->ikSolver == SolverIK::dynamical) {
        if (!(config.check("useDirectBaseMeasurement")
              && config.find("useDirectBaseMeasurement").isBool())) {
            yError() << LogPrefix << "useDirectBaseMeasurement option not found or not valid";
            return false;
        }
        if (!(config.check("linVelTargetWeight")
              && config.find("linVelTargetWeight").isFloat64())) {
            yError() << LogPrefix << "linVelTargetWeight option not found or not valid";
            return false;
        }

        if (!(config.check("angVelTargetWeight")
              && config.find("angVelTargetWeight").isFloat64())) {
            yError() << LogPrefix << "angVelTargetWeight option not found or not valid";
            return false;
        }

        if (config.check("inverseVelocityKinematicsSolver")
            && config.find("inverseVelocityKinematicsSolver").isString()) {
            pImpl->inverseVelocityKinematicsSolver =
                config.find("inverseVelocityKinematicsSolver").asString();
        }
        else {
            pImpl->inverseVelocityKinematicsSolver = "moorePenrose";
            yInfo() << LogPrefix << "Using default inverse velocity kinematics solver";
        }

        pImpl->useDirectBaseMeasurement = config.find("useDirectBaseMeasurement").asBool();
        pImpl->linVelTargetWeight = config.find("linVelTargetWeight").asFloat64();
        pImpl->angVelTargetWeight = config.find("angVelTargetWeight").asFloat64();
        pImpl->costRegularization = config.find("costRegularization").asFloat64();
    }

    if (pImpl->ikSolver == SolverIK::pairwised) {
        if (!(config.check("ikPoolSizeOption")
              && (config.find("ikPoolSizeOption").isString()
                  || config.find("ikPoolSizeOption").isInt32()))) {
            yError() << LogPrefix << "ikPoolOption option not found or not valid";
            return false;
        }

        // Get ikPoolSizeOption
        if (config.find("ikPoolSizeOption").isString()
            && config.find("ikPoolSizeOption").asString() == "auto") {
            yInfo() << LogPrefix << "Using " << std::thread::hardware_concurrency()
                    << " available logical threads for ik pool";
            pImpl->ikPoolSize = static_cast<int>(std::thread::hardware_concurrency());
        }
        else if (config.find("ikPoolSizeOption").isInt32()) {
            pImpl->ikPoolSize = config.find("ikPoolSizeOption").asInt32();
        }

        // The pairwised IK will always use the measured base pose and velocity for the base link
        if (config.check("useDirectBaseMeasurement")
            && config.find("useDirectBaseMeasurement").isBool()
            && !config.find("useDirectBaseMeasurement").asBool()) {
            yWarning() << LogPrefix
                       << "useDirectBaseMeasurement is required from Pair-Wised IK. Assuming its "
                          "value to be true";
        }
        pImpl->useDirectBaseMeasurement = true;
    }

    if (pImpl->ikSolver == SolverIK::dynamical) {

        if (!(config.check("dynamicalIKMeasuredVelocityGainLinRot")
              && config.find("dynamicalIKMeasuredVelocityGainLinRot").isList()
              && config.find("dynamicalIKMeasuredVelocityGainLinRot").asList()->size() == 2)) {
            yError() << LogPrefix
                     << "dynamicalIKMeasuredVelocityGainLinRot option not found or not valid";
            return false;
        }

        if (!(config.check("dynamicalIKCorrectionGainsLinRot")
              && config.find("dynamicalIKCorrectionGainsLinRot").isList()
              && config.find("dynamicalIKCorrectionGainsLinRot").asList()->size() == 2)) {
            yError() << LogPrefix
                     << "dynamicalIKCorrectionGainsLinRot option not found or not valid";
            return false;
        }

        if (config.check("dynamicalIKJointVelocityLimit")
            && config.find("dynamicalIKJointVelocityLimit").isFloat64()) {
            pImpl->dynamicalIKJointVelocityLimit =
                config.find("dynamicalIKJointVelocityLimit").asFloat64();
        }
        else {
            pImpl->dynamicalIKJointVelocityLimit =
                1000.0; // if no limits given for a joint we put 1000.0 rad/sec, which is very high
        }

        yarp::os::Bottle* dynamicalIKMeasuredVelocityGainLinRot =
            config.find("dynamicalIKMeasuredVelocityGainLinRot").asList();
        yarp::os::Bottle* dynamicalIKCorrectionGainsLinRot =
            config.find("dynamicalIKCorrectionGainsLinRot").asList();
        pImpl->dynamicalIKMeasuredLinearVelocityGain =
            dynamicalIKMeasuredVelocityGainLinRot->get(0).asFloat64();
        pImpl->dynamicalIKMeasuredAngularVelocityGain =
            dynamicalIKMeasuredVelocityGainLinRot->get(1).asFloat64();
        pImpl->dynamicalIKLinearCorrectionGain =
            dynamicalIKCorrectionGainsLinRot->get(0).asFloat64();
        pImpl->dynamicalIKAngularCorrectionGain =
            dynamicalIKCorrectionGainsLinRot->get(1).asFloat64();
    }

    // ===================================
    // PRINT CURRENT CONFIGURATION OPTIONS
    // ===================================

    yInfo() << LogPrefix << "*** ===================================";
    yInfo() << LogPrefix << "*** Period                            :" << pImpl->period;
    yInfo() << LogPrefix << "*** Urdf file name                    :" << urdfFileName;
    yInfo() << LogPrefix << "*** Ik solver                         :" << solverName;
    yInfo() << LogPrefix
            << "*** Use Directly base measurement    :" << pImpl->useDirectBaseMeasurement;
    if (pImpl->ikSolver == SolverIK::pairwised || pImpl->ikSolver == SolverIK::global) {
        yInfo() << LogPrefix << "*** Allow IK failures                 :" << pImpl->allowIKFailures;
        yInfo() << LogPrefix << "*** Max IK iterations                 :" << pImpl->maxIterationsIK;
        yInfo() << LogPrefix << "*** Cost Tolerance                    :" << pImpl->costTolerance;
        yInfo() << LogPrefix
                << "*** IK Solver Name                    :" << pImpl->linearSolverName;
        yInfo() << LogPrefix << "*** Position target weight            :" << pImpl->posTargetWeight;
        yInfo() << LogPrefix << "*** Rotation target weight            :" << pImpl->rotTargetWeight;
        yInfo() << LogPrefix
                << "*** Cost regularization              :" << pImpl->costRegularization;
        yInfo() << LogPrefix << "*** Size of thread pool               :" << pImpl->ikPoolSize;
    }
    if (pImpl->ikSolver == SolverIK::dynamical) {
        yInfo() << LogPrefix << "*** Measured Linear velocity gain     :"
                << pImpl->dynamicalIKMeasuredLinearVelocityGain;
        yInfo() << LogPrefix << "*** Measured Angular velocity gain    :"
                << pImpl->dynamicalIKMeasuredAngularVelocityGain;
        yInfo() << LogPrefix << "*** Linear correction gain            :"
                << pImpl->dynamicalIKLinearCorrectionGain;
        yInfo() << LogPrefix << "*** Angular correction gain           :"
                << pImpl->dynamicalIKAngularCorrectionGain;
        yInfo() << LogPrefix
                << "*** Cost regularization              :" << pImpl->costRegularization;
        yInfo() << LogPrefix
                << "*** Joint velocity limit             :" << pImpl->dynamicalIKJointVelocityLimit;
    }
    if (pImpl->ikSolver == SolverIK::dynamical || pImpl->ikSolver == SolverIK::global) {
        yInfo() << LogPrefix << "*** Inverse Velocity Kinematics solver:"
                << pImpl->inverseVelocityKinematicsSolver;
    }
    yInfo() << LogPrefix << "*** ===================================";

    // ==========================
    // INITIALIZE THE HUMAN MODEL
    // ==========================

    auto& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string urdfFilePath = rf.findFile(urdfFileName);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << config.find("urdf").asString();
        return false;
    }

    iDynTree::ModelLoader modelLoader;
    if (pImpl->jointList.empty()) {
        if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
            yError() << LogPrefix << "Failed to load model" << urdfFilePath;
            return false;
        }
    }
    else {
        if (!modelLoader.loadReducedModelFromFile(urdfFilePath, pImpl->jointList)
            || !modelLoader.isValid()) {
            yError() << LogPrefix << "Failed to load model" << urdfFilePath;
            return false;
        }
    }

    yInfo() << LogPrefix << "----------------------------------------" << modelLoader.isValid();
    yInfo() << LogPrefix << modelLoader.model().toString();
    yInfo() << LogPrefix << modelLoader.model().getNrOfLinks()
            << " , joints: " << modelLoader.model().getNrOfJoints();

    yInfo() << LogPrefix << "base link: "
            << modelLoader.model().getLinkName(modelLoader.model().getDefaultBaseLink());

    // ====================
    // INITIALIZE VARIABLES
    // ====================

    // Get the model from the loader
    pImpl->humanModel = modelLoader.model();

    // Set gravity
    pImpl->worldGravity.zero();
    pImpl->worldGravity(2) = -9.81;

    // Initialize kinDyn computation
    pImpl->kinDynComputations =
        std::unique_ptr<iDynTree::KinDynComputations>(new iDynTree::KinDynComputations());
    pImpl->kinDynComputations->loadRobotModel(modelLoader.model());
    pImpl->kinDynComputations->setFloatingBase(pImpl->floatingBaseFrame);

    // =========================
    // INITIALIZE JOINTS BUFFERS
    // =========================

    // Get the number of joints accordingly to the model
    const size_t nrOfDOFs = pImpl->humanModel.getNrOfDOFs();

    pImpl->solution.jointPositions.resize(nrOfDOFs);
    pImpl->solution.jointVelocities.resize(nrOfDOFs);

    pImpl->jointConfigurationSolution.resize(nrOfDOFs);
    pImpl->jointConfigurationSolution.zero();

    pImpl->jointVelocitiesSolution.resize(nrOfDOFs);
    pImpl->jointVelocitiesSolution.zero();

    pImpl->jointCalibrationSolution.resize(nrOfDOFs);

    // Fill iDynTreeVector with 0 if calibration configuration list is empty
    if (calibrationJointConfiguration.empty()) {
        pImpl->jointCalibrationSolution.zero();
    }
    else if (calibrationJointConfiguration.size() != nrOfDOFs) {
        yError() << LogPrefix << "calibrationJointConfiguration param size ("
                 << calibrationJointConfiguration.size()
                 << ") must match the number of DOFs of the model (" << nrOfDOFs << "). ";
        return false;
    }
    else {
        for (int idx = 0; idx < calibrationJointConfiguration.size(); idx++) {
            pImpl->jointCalibrationSolution.setVal(idx, calibrationJointConfiguration[idx]);
        }
    }

    // =======================
    // INITIALIZE BASE BUFFERS
    // =======================
    pImpl->baseTransformSolution = iDynTree::Transform::Identity();
    pImpl->baseVelocitySolution.zero();

    // ================================================
    // INITIALIZE CUSTOM CONSTRAINTS FOR INTEGRATION-IK
    // ================================================

    pImpl->customConstraintMatrix.resize(0, 0);
    pImpl->customConstraintVariables.resize(0);
    pImpl->customConstraintLowerBound.resize(0);
    pImpl->customConstraintUpperBound.resize(0);
    pImpl->customConstraintVariablesIndex.resize(0);
    pImpl->custom_jointsVelocityLimitsNames.resize(0);
    pImpl->custom_jointsVelocityLimitsValues.resize(0);
    pImpl->custom_jointsVelocityLimitsIndexes.resize(0);
    pImpl->k_u = 0.5;
    pImpl->k_l = 0.5;

    if (config.check("CUSTOM_CONSTRAINTS")) {

        yarp::os::Bottle& constraintGroup = config.findGroup("CUSTOM_CONSTRAINTS");
        if (constraintGroup.isNull()) {
            yError() << LogPrefix << "Failed to find group CUSTOM_CONSTRAINTS";
            return false;
        }
        if (pImpl->inverseVelocityKinematicsSolver != "QP"
            || pImpl->ikSolver != SolverIK::dynamical) {
            yWarning() << LogPrefix
                       << "'CUSTOM_CONSTRAINTS' group option is available only if "
                          "'ikSolver==dynamical' & 'inverseVelocityKinematicsSolver==QP'. \n "
                          "Currently, you are NOT using the customized constraint group.";
        }

        yInfo() << "==================>>>>>>> constraint group: " << constraintGroup.size();

        for (size_t i = 1; i < constraintGroup.size(); i++) {
            yInfo() << "group " << i;
            if (!(constraintGroup.get(i).isList()
                  && constraintGroup.get(i).asList()->size() == 2)) {
                yError() << LogPrefix
                         << "Childs of CUSTOM_CONSTRAINTS must be lists of two elements";
                return false;
            }
            else {
                yInfo() << "Everything is fine...";
            }
            yarp::os::Bottle* constraintList = constraintGroup.get(i).asList();
            std::string constraintKey = constraintList->get(0).asString();
            yarp::os::Bottle* constraintListContent = constraintList->get(1).asList();
            yInfo() << constraintKey;
            if (constraintKey == "custom_joints_velocity_limits_names") {

                for (size_t i = 0; i < constraintListContent->size(); i++) {
                    pImpl->custom_jointsVelocityLimitsNames.push_back(
                        constraintListContent->get(i).asString());
                }
                yInfo() << "custom_joints_velocity_limits_names: ";
                for (size_t i = 0; i < pImpl->custom_jointsVelocityLimitsNames.size(); i++) {
                    yInfo() << pImpl->custom_jointsVelocityLimitsNames[i];
                }
            } // another option
            else if (constraintKey == "custom_joints_velocity_limits_values") {
                pImpl->custom_jointsVelocityLimitsValues.resize(constraintListContent->size());
                for (size_t i = 0; i < constraintListContent->size(); i++) {
                    pImpl->custom_jointsVelocityLimitsValues.setVal(
                        i, constraintListContent->get(i).asFloat64());
                }
                yInfo() << "custom_joints_velocity_limits_values: ";
                for (size_t i = 0; i < pImpl->custom_jointsVelocityLimitsValues.size(); i++) {
                    yInfo() << pImpl->custom_jointsVelocityLimitsValues.getVal(i);
                }
            } // another option
            else if (constraintKey == "custom_constraint_variables") {

                for (size_t i = 0; i < constraintListContent->size(); i++) {
                    pImpl->customConstraintVariables.push_back(
                        constraintListContent->get(i).asString());
                }
                yInfo() << "custom_constraint_variables: ";
                for (size_t i = 0; i < pImpl->customConstraintVariables.size(); i++) {
                    yInfo() << pImpl->customConstraintVariables[i];
                }
            } // another option
            else if (constraintKey == "custom_constraint_matrix") {
                // pImpl->customConstraintMatrix.resize(constraintListContent->size());
                for (size_t i = 0; i < constraintListContent->size(); i++) {
                    yarp::os::Bottle* innerLoop = constraintListContent->get(i).asList();
                    if (i == 0) {
                        pImpl->customConstraintMatrix.resize(constraintListContent->size(),
                                                             innerLoop->size());
                    }
                    for (size_t j = 0; j < innerLoop->size(); j++) {
                        pImpl->customConstraintMatrix.setVal(i, j, innerLoop->get(j).asFloat64());
                    }
                }
                yInfo() << "Constraint matrix: ";
                for (size_t i = 0; i < pImpl->customConstraintMatrix.rows(); i++) {
                    for (size_t j = 0; j < pImpl->customConstraintMatrix.cols(); j++) {
                        std::cout << pImpl->customConstraintMatrix.getVal(i, j) << " ";
                    }
                    std::cout << std::endl;
                }
            } // another option
            else if (constraintKey == "custom_constraint_upper_bound") {

                pImpl->customConstraintUpperBound.resize(constraintListContent->size());
                for (size_t i = 0; i < constraintListContent->size(); i++) {
                    pImpl->customConstraintUpperBound.setVal(
                        i, constraintListContent->get(i).asFloat64());
                }
                yInfo() << "custom_constraint_upper_bound: ";
                for (size_t i = 0; i < pImpl->customConstraintUpperBound.size(); i++) {
                    yInfo() << pImpl->customConstraintUpperBound.getVal(i);
                }
            } // another option
            else if (constraintKey == "custom_constraint_lower_bound") {
                pImpl->customConstraintLowerBound.resize(constraintListContent->size());
                for (size_t i = 0; i < constraintListContent->size(); i++) {
                    pImpl->customConstraintLowerBound.setVal(
                        i, constraintListContent->get(i).asFloat64());
                }
                yInfo() << "custom_constraint_lower_bound: ";
                for (size_t i = 0; i < pImpl->customConstraintLowerBound.size(); i++) {
                    yInfo() << pImpl->customConstraintLowerBound.getVal(i);
                }
            } // another option
            else if (constraintKey == "base_velocity_limit_upper_buond") {
                if (constraintListContent->size() != 6) {
                    yError() << "the base velocity limit should have size of 6.";
                    return false;
                }
                pImpl->baseVelocityUpperLimit.resize(6);
                for (size_t i = 0; i < constraintListContent->size(); i++) {
                    pImpl->baseVelocityUpperLimit.setVal(i,
                                                         constraintListContent->get(i).asFloat64());
                }
                yInfo() << "base_velocity_limit_upper_buond: ";
                for (size_t i = 0; i < pImpl->baseVelocityUpperLimit.size(); i++) {
                    yInfo() << pImpl->baseVelocityUpperLimit.getVal(i);
                }
            } // another option
            else if (constraintKey == "base_velocity_limit_lower_buond") {
                if (constraintListContent->size() != 6) {
                    yError() << "the base velocity limit should have size of 6.";
                    return false;
                }
                pImpl->baseVelocityLowerLimit.resize(6);
                for (size_t i = 0; i < constraintListContent->size(); i++) {
                    pImpl->baseVelocityLowerLimit.setVal(i,
                                                         constraintListContent->get(i).asFloat64());
                }
                yInfo() << "base_velocity_limit_lower_buond: ";
                for (size_t i = 0; i < pImpl->baseVelocityLowerLimit.size(); i++) {
                    yInfo() << pImpl->baseVelocityLowerLimit.getVal(i);
                }
            } // another option
            else if (constraintKey == "k_u") {
                if (constraintGroup.check("k_u") && constraintGroup.find("k_u").isFloat64()) {
                    pImpl->k_u = constraintGroup.find("k_u").asFloat64();
                    yInfo() << "k_u: " << pImpl->k_u;
                }
            } // another option
            else if (constraintKey == "k_l") {
                if (constraintGroup.check("k_l") && constraintGroup.find("k_l").isFloat64()) {
                    pImpl->k_l = constraintGroup.find("k_l").asFloat64();
                    yInfo() << "k_l: " << pImpl->k_l;
                }
            } // another option
            else {
                yError() << LogPrefix << "the parameter key is not defined: " << constraintKey;
                return false;
            }
        }
    }
    else {
        yInfo() << "CUSTOM CONSTRAINTS are not defined in xml file.";
    }

    // set base velocity constraint to zero if the base is fixed
    if (pImpl->useFixedBase) {
        pImpl->baseVelocityLowerLimit.resize(6);
        pImpl->baseVelocityLowerLimit.zero();
        pImpl->baseVelocityUpperLimit.resize(6);
        pImpl->baseVelocityUpperLimit.zero();

        yInfo() << "Using fixed base model, base velocity limits are set to zero";
    }

    // check sizes
    if (pImpl->custom_jointsVelocityLimitsNames.size()
        != pImpl->custom_jointsVelocityLimitsValues.size()) {
        yError() << "the joint velocity limits name and value size are not equal";
        return false;
    }
    if ((pImpl->customConstraintUpperBound.size() != pImpl->customConstraintLowerBound.size())
        && (pImpl->customConstraintLowerBound.size() != pImpl->customConstraintMatrix.rows())) {
        yError() << "the number of lower bound (" << pImpl->customConstraintLowerBound.size()
                 << "), upper buond(" << pImpl->customConstraintUpperBound.size()
                 << "), and cosntraint matrix rows(" << pImpl->customConstraintMatrix.rows()
                 << ") are not equal";

        return false;
    }
    if ((pImpl->customConstraintVariables.size() != pImpl->customConstraintMatrix.cols())) {
        yError() << "the number of constraint variables ("
                 << pImpl->customConstraintVariables.size() << "), and cosntraint matrix columns ("
                 << pImpl->customConstraintMatrix.cols() << ") are not equal";
        return false;
    }
    yInfo() << "******* DOF: " << modelLoader.model().getNrOfDOFs();
    for (size_t i = 0; i < pImpl->custom_jointsVelocityLimitsNames.size(); i++) {
        pImpl->custom_jointsVelocityLimitsIndexes.push_back(
            modelLoader.model().getJointIndex(pImpl->custom_jointsVelocityLimitsNames[i]));
        yInfo() << pImpl->custom_jointsVelocityLimitsNames[i] << " : "
                << pImpl->custom_jointsVelocityLimitsIndexes[i];
    }
    for (size_t i = 0; i < pImpl->customConstraintVariables.size(); i++) {
        pImpl->customConstraintVariablesIndex.push_back(
            modelLoader.model().getJointIndex(pImpl->customConstraintVariables[i]));
        yInfo() << pImpl->customConstraintVariables[i] << " : "
                << pImpl->customConstraintVariablesIndex[i];
    }

    // ==================================================================
    // CREATE LINK PAIR VECTOR FOR SECONDARY CALIBRATION AND PAIRWISED IK
    // ==================================================================

    if (!pImpl->createLinkPairs()) {
        askToStop();
        return false;
    }

    // ====================================
    // INITIALIZE INVERSE KINEMATICS SOLVER
    // ====================================

    if (pImpl->ikSolver == SolverIK::pairwised) {
        if (!pImpl->initializePairwisedInverseKinematicsSolver()) {
            askToStop();
            return false;
        }
    }
    else if (pImpl->ikSolver == SolverIK::global) {
        if (!pImpl->initializeGlobalInverseKinematicsSolver()) {
            askToStop();
            return false;
        }
    }
    else if (pImpl->ikSolver == SolverIK::dynamical) {
        if (!pImpl->initializeDynamicalInverseKinematicsSolver()) {
            askToStop();
            return false;
        }
    }

    // ===================
    // INITIALIZE RPC PORT
    // ===================

    std::string rpcPortName;
    if (!(config.check("rpcPortPrefix") && config.find("rpcPortPrefix").isString())) {
        rpcPortName = "/" + DeviceName + "/rpc:i";
    }
    else {
        rpcPortName = "/" + config.find("rpcPortPrefix").asString() + "/" + DeviceName + "/rpc:i";
    }

    if (!pImpl->rpcPort.open(rpcPortName)) {
        yError() << LogPrefix << "Unable to open rpc port " << rpcPortName;
        return false;
    }

    // Set rpc port reader
    pImpl->rpcPort.setReader(*pImpl->commandPro);

    return true;
}

bool HumanStateProvider::close()
{
    return true;
}

void HumanStateProvider::run()
{
    // Update the target from wearable interface
    if (!pImpl->updateWearableTargets()) {
        yError() << LogPrefix << "Failed to get link transforms from input data";
        askToStop();
        return;
    }

    // Solve Inverse Kinematics and Inverse Velocity Problems
    auto tick = std::chrono::high_resolution_clock::now();
    bool inverseKinematicsFailure;
    if (pImpl->ikSolver == SolverIK::pairwised) {
        inverseKinematicsFailure = !(pImpl->solvePairwisedInverseKinematicsSolver());
    }
    else if (pImpl->ikSolver == SolverIK::global) {
        inverseKinematicsFailure = !pImpl->solveGlobalInverseKinematicsSolver();
    }
    else if (pImpl->ikSolver == SolverIK::dynamical) {
        inverseKinematicsFailure = !pImpl->solveDynamicalInverseKinematics();
    }

    // check if inverse kinematics failed
    if (inverseKinematicsFailure) {
        if (pImpl->allowIKFailures) {
            yWarning() << LogPrefix << "IK failed, keeping the previous solution";
            return;
        }
        else {
            yError() << LogPrefix << "Failed to solve IK";
            askToStop();
        }
        return;
    }

    auto tock = std::chrono::high_resolution_clock::now();
    yTrace() << LogPrefix << "IK took"
             << std::chrono::duration_cast<std::chrono::milliseconds>(tock - tick).count() << "ms";

    // If useDirectBaseMeasurement is true, directly use the measured base pose and velocity. If
    // useFixedBase is also enabled, identity transform and zero velocity will be used.
    if (pImpl->useFixedBase) {
        pImpl->baseTransformSolution = iDynTree::Transform::Identity();
        pImpl->baseVelocitySolution.zero();
    }
    else if (pImpl->useDirectBaseMeasurement) {
        pImpl->baseTransformSolution = pImpl->linkTransformMatrices.at(pImpl->floatingBaseFrame);
        pImpl->baseVelocitySolution = pImpl->linkVelocities.at(pImpl->floatingBaseFrame);
    }

    // CoM position and velocity
    std::array<double, 3> CoM_position, CoM_velocity;
    iDynTree::KinDynComputations* kindyncomputations = pImpl->kinDynComputations.get();
    CoM_position = {kindyncomputations->getCenterOfMassPosition().getVal(0),
                    kindyncomputations->getCenterOfMassPosition().getVal(1),
                    kindyncomputations->getCenterOfMassPosition().getVal(2)};

    CoM_velocity = {kindyncomputations->getCenterOfMassVelocity().getVal(0),
                    kindyncomputations->getCenterOfMassVelocity().getVal(1),
                    kindyncomputations->getCenterOfMassVelocity().getVal(2)};

    // Expose IK solution for IHumanState
    {
        std::lock_guard<std::mutex> lock(pImpl->mutex);

        for (unsigned i = 0; i < pImpl->jointConfigurationSolution.size(); ++i) {
            pImpl->solution.jointPositions[i] = pImpl->jointConfigurationSolution.getVal(i);
            pImpl->solution.jointVelocities[i] = pImpl->jointVelocitiesSolution.getVal(i);
        }

        pImpl->solution.basePosition = {pImpl->baseTransformSolution.getPosition().getVal(0),
                                        pImpl->baseTransformSolution.getPosition().getVal(1),
                                        pImpl->baseTransformSolution.getPosition().getVal(2)};

        pImpl->solution.baseOrientation = {
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(0),
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(1),
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(2),
            pImpl->baseTransformSolution.getRotation().asQuaternion().getVal(3)};

        // Use measured base frame velocity
        pImpl->solution.baseVelocity = {pImpl->baseVelocitySolution.getVal(0),
                                        pImpl->baseVelocitySolution.getVal(1),
                                        pImpl->baseVelocitySolution.getVal(2),
                                        pImpl->baseVelocitySolution.getVal(3),
                                        pImpl->baseVelocitySolution.getVal(4),
                                        pImpl->baseVelocitySolution.getVal(5)};
        // CoM position and velocity
        pImpl->solution.CoMPosition = {CoM_position[0], CoM_position[1], CoM_position[2]};
        pImpl->solution.CoMVelocity = {CoM_velocity[0], CoM_velocity[1], CoM_velocity[2]};
    }

    // Check for rpc command status and apply command
    if (pImpl->commandPro->cmdStatus != rpcCommand::empty) {

        // Apply rpc command
        if (!pImpl->applyRpcCommand()) {
            yWarning() << LogPrefix << "Failed to execute the rpc command";
        }

        // reset the rpc internal status
        {
            std::lock_guard<std::mutex> lock(pImpl->mutex);
            pImpl->commandPro->resetInternalVariables();
        }
    }

    // compute the inverse kinematic errors (currently the result is unused, but it may be used for
    // evaluating the IK performance)
    // pImpl->computeLinksOrientationErrors(pImpl->linkTransformMatrices,
    //                                      pImpl->jointConfigurationSolution,
    //                                      pImpl->baseTransformSolution,
    //                                      pImpl->linkErrorOrientations);
    // pImpl->computeLinksAngularVelocityErrors(pImpl->linkVelocities,
    //                                          pImpl->jointConfigurationSolution,
    //                                          pImpl->baseTransformSolution,
    //                                          pImpl->jointVelocitiesSolution,
    //                                          pImpl->baseVelocitySolution,
    //                                          pImpl->linkErrorAngularVelocities);
}

void HumanStateProvider::impl::ereaseTargetCalibration(const hde::TargetName& targetName)
{
    wearableTargets[targetName].get()->clearCalibrationMatrices();
}

void HumanStateProvider::impl::ereaseTargetsCalibration()
{
    for (auto wearableTargetEntry : wearableTargets) {
        wearableTargetEntry.second.get()->clearCalibrationMatrices();
    }
}

void HumanStateProvider::impl::selectChainJointsAndLinksForSecondaryCalibration(
    const std::string& linkName,
    const std::string& childLinkName,
    std::vector<iDynTree::JointIndex>& jointZeroIndices,
    std::vector<iDynTree::LinkIndex>& linkToCalibrateIndices)
{
    if (childLinkName == "") {
        // Select the chosen link [linkName] and the joints between the link and its parend link

        // add link to [linkToCalibrateIndices]
        linkToCalibrateIndices.push_back(kinDynComputations->model().getLinkIndex(linkName));

        // add joints to [jointZeroIndices]
        for (auto pairInfo = linkPairs.begin(); pairInfo != linkPairs.end(); pairInfo++) {
            if (pairInfo->parentFrameName == linkName) {
                for (size_t pairModelJointIndex = 0;
                     pairModelJointIndex < pairInfo->pairModel.getNrOfJoints();
                     pairModelJointIndex++) {
                    std::string jointName = pairInfo->pairModel.getJointName(pairModelJointIndex);
                    jointZeroIndices.push_back(
                        kinDynComputations->model().getJointIndex(jointName));
                }
                break;
            }
        }
    }
    else {
        // Create chain between the given parent link [linkName] and the child link [childLinkName]
        // and select the joints and the links involved in the chain

        // create reduced model between parent and child frame
        iDynTree::Model chainModel;
        getReducedModel(kinDynComputations->model(), linkName, childLinkName, chainModel);

        // add links found in the submodel to [linkToCalibrateIndices]
        // TODO missing fake links that are frames
        for (size_t chainModelLinkIndex = 0; chainModelLinkIndex < chainModel.getNrOfLinks();
             chainModelLinkIndex++) {
            std::string chainLinkName = chainModel.getLinkName(chainModelLinkIndex);
            linkToCalibrateIndices.push_back(
                kinDynComputations->model().getLinkIndex(chainLinkName));
        }

        // add joints found in the submodel to [jointZeroIndices]
        for (size_t chainModelJointIndex = 0; chainModelJointIndex < chainModel.getNrOfJoints();
             chainModelJointIndex++) {
            std::string jointName = chainModel.getJointName(chainModelJointIndex);
            jointZeroIndices.push_back(kinDynComputations->model().getJointIndex(jointName));
        }
    }
}

void HumanStateProvider::impl::computeSecondaryCalibrationRotationsForChain(
    const std::vector<iDynTree::JointIndex>& jointZeroIndices,
    const iDynTree::Transform& refLinkForCalibrationTransform,
    const std::vector<iDynTree::LinkIndex>& linkToCalibrateIndices,
    const hde::TargetName& refTargetForCalibrationName)
{
    // initialize vectors
    iDynTree::VectorDynSize jointPos(jointConfigurationSolution);
    jointPos.zero();
    iDynTree::VectorDynSize jointVel(jointVelocitiesSolution);
    jointVel.zero();
    iDynTree::Twist baseVel;
    baseVel.zero();

    {
        std::lock_guard<std::mutex> lock(mutexFlagIntegrator);
        resetIntegrator = true;
    }

    // TODO check which value to give to the base (before we were using the base target measurement)
    kinDynComputations->setRobotState(
        iDynTree::Transform::Identity(), jointPos, baseVel, jointVel, worldGravity);

    // If needed compute world calibration matrix
    // in this case the same world calibration transform i used for all the targets
    iDynTree::Transform secondaryCalibrationWorld = iDynTree::Transform::Identity();
    if (refTargetForCalibrationName != "") {
        std::string linkName = wearableTargets[refTargetForCalibrationName].get()->modelLinkName;
        iDynTree::Transform linkForCalibrationTransform =
            kinDynComputations->getWorldTransform(linkName);
        secondaryCalibrationWorld =
            refLinkForCalibrationTransform * linkForCalibrationTransform.inverse();
    }

    for (auto wearableTargetEntry : wearableTargets) {
        hde::TargetName targetName = wearableTargetEntry.first;
        ModelLinkName linkName = wearableTargetEntry.second->modelLinkName;
        iDynTree::LinkIndex linkIndex = kinDynComputations->model().getLinkIndex(linkName);

        std::cerr << "target: " << targetName << std::endl;
        std::cerr << "link index" << linkIndex << std::endl;

        if (std::find(linkToCalibrateIndices.begin(), linkToCalibrateIndices.end(), linkIndex)
            != linkToCalibrateIndices.end()) {
            std::cerr << "link found" << std::endl;
            wearableTargetEntry.second->clearSecondaryCalibrationMatrix();
            wearableTargetEntry.second->calibrationMeasurementToLink.setRotation(
                wearableTargetEntry.second->getCalibratedRotation().inverse()
                * kinDynComputations->getWorldTransform(linkName).getRotation());
            wearableTargetEntry.second->calibrationMeasurementToLink.setPosition(
                ((kinDynComputations->getWorldTransform(linkName).getPosition()
                  - wearableTargetEntry.second->calibrationWorldToMeasurementWorld.getPosition())
                     .changeCoordinateFrame(
                         wearableTargetEntry.second->calibrationWorldToMeasurementWorld
                             .getRotation()
                             .inverse())
                 - iDynTree::Position(wearableTargetEntry.second->position))
                    .changeCoordinateFrame(wearableTargetEntry.second->rotation.inverse()));

            wearableTargetEntry.second->calibrationWorldToMeasurementWorld =
                secondaryCalibrationWorld
                * wearableTargetEntry.second->calibrationWorldToMeasurementWorld;

            yInfo() << LogPrefix << "Sensor to Link calibration rotation for " << targetName
                    << " is set";
        }
    }
}

bool HumanStateProvider::impl::applyRpcCommand()
{
    // check is the choosen links are valid
    hde::TargetName targetName = commandPro->parentLinkName;
    hde::TargetName childTargetName = commandPro->childLinkName;
    if (!(targetName == "") && (wearableTargets.find(targetName) == wearableTargets.end())) {
        yWarning() << LogPrefix << "Target " << targetName
                   << " choosen for secondaty calibration is not valid";
        return false;
    }
    if (!(childTargetName == "")
        && (wearableTargets.find(childTargetName) == wearableTargets.end())) {
        yWarning() << LogPrefix << "Target " << childTargetName
                   << " choosen for secondaty calibration is not valid";
        return false;
    }

    // initialize buffer variable for calibration
    std::vector<iDynTree::JointIndex> jointZeroIndices;
    std::vector<iDynTree::LinkIndex> linkToCalibrateIndices;
    iDynTree::Rotation secondaryCalibrationRotation;

    switch (commandPro->cmdStatus) {
        case rpcCommand::resetAll: {
            ereaseTargetsCalibration();
            break;
        }
        case rpcCommand::resetCalibration: {
            ereaseTargetCalibration(targetName);
            break;
        }
        case rpcCommand::calibrateAll: {
            // Select all the links and the joints
            // add all the links of the model to [linkToCalibrateIndices]
            linkToCalibrateIndices.resize(kinDynComputations->getNrOfLinks());
            std::iota(linkToCalibrateIndices.begin(), linkToCalibrateIndices.end(), 0);

            // add all the joints of the model to [jointZeroIndices]
            jointZeroIndices.resize(kinDynComputations->getNrOfDegreesOfFreedom());
            std::iota(jointZeroIndices.begin(), jointZeroIndices.end(), 0);

            // Compute secondary calibration for the selected links setting to zero the given joints
            computeSecondaryCalibrationRotationsForChain(
                jointZeroIndices, iDynTree::Transform::Identity(), linkToCalibrateIndices, "");
            break;
        }
        case rpcCommand::calibrateAllWorldYaw: {
            // Select all the links and the joints
            linkToCalibrateIndices.resize(kinDynComputations->getNrOfLinks());
            std::iota(linkToCalibrateIndices.begin(), linkToCalibrateIndices.end(), 0);

            // add all the joints of the model to [jointZeroIndices]
            jointZeroIndices.resize(kinDynComputations->getNrOfDegreesOfFreedom());
            std::iota(jointZeroIndices.begin(), jointZeroIndices.end(), 0);

            // initialize vectors
            iDynTree::VectorDynSize jointPos(jointConfigurationSolution);
            iDynTree::VectorDynSize jointVel(jointVelocitiesSolution);
            jointVel.zero();
            iDynTree::Twist baseVel;
            baseVel.zero();

            for (auto const& jointZeroIdx : jointZeroIndices) {

                jointPos.setVal(jointZeroIdx, jointCalibrationSolution.getVal(jointZeroIdx));
            }

            kinDynComputations->setRobotState(
                iDynTree::Transform::Identity(), jointPos, baseVel, jointVel, worldGravity);

            for (auto wearableTargetEntry : wearableTargets) {
                hde::TargetName targetName = wearableTargetEntry.first;
                ModelLinkName linkName = wearableTargetEntry.second->modelLinkName;
                iDynTree::LinkIndex linkIndex = kinDynComputations->model().getLinkIndex(linkName);

                if (std::find(
                        linkToCalibrateIndices.begin(), linkToCalibrateIndices.end(), linkIndex)
                    != linkToCalibrateIndices.end()) {
                    wearableTargetEntry.second->clearWorldCalibrationMatrix();

                    iDynTree::Vector3 rpyOffsetTransform =
                        iDynTree::Rotation(
                            kinDynComputations->getWorldTransform(linkName).getRotation()
                            * wearableTargetEntry.second->getCalibratedRotation().inverse())
                            .asRPY();
                    wearableTargetEntry.second->calibrationWorldToMeasurementWorld.setRotation(
                        iDynTree::Rotation::RotZ(rpyOffsetTransform.getVal(2)));
                }
            }
        }
        case rpcCommand::calibrateAllWithWorld: {
            // Check if the chose baseLink exist in the model
            hde::TargetName refTargetForCalibrationName = commandPro->refLinkName;
            if ((wearableTargets.find(refTargetForCalibrationName) == wearableTargets.end())) {
                yWarning() << LogPrefix << "Target " << refTargetForCalibrationName
                           << " choosen as base for secondaty calibration is not valid";
                return false;
            }

            // Select all the links and the joints
            // add all the links of the model to [linkToCalibrateIndices]
            linkToCalibrateIndices.resize(kinDynComputations->getNrOfLinks());
            std::iota(linkToCalibrateIndices.begin(), linkToCalibrateIndices.end(), 0);

            // add all the joints of the model to [jointZeroIndices]
            jointZeroIndices.resize(kinDynComputations->getNrOfDegreesOfFreedom());
            std::iota(jointZeroIndices.begin(), jointZeroIndices.end(), 0);

            // Compute secondary calibration for the selected links setting to zero the given joints
            computeSecondaryCalibrationRotationsForChain(jointZeroIndices,
                                                         iDynTree::Transform::Identity(),
                                                         linkToCalibrateIndices,
                                                         refTargetForCalibrationName);
            break;
        }
        case rpcCommand::calibrateRelativeLink: {
            ereaseTargetCalibration(childTargetName);
            // Compute the relative transform at zero configuration
            // setting to zero all the joints
            iDynTree::VectorDynSize jointPos;
            jointPos.resize(jointConfigurationSolution.size());
            jointPos.zero();
            kinDynComputations->setJointPos(jointPos);
            std::string childLinkName = wearableTargets[childTargetName].get()->modelLinkName;
            std::string parentLinkName = wearableTargets[targetName].get()->modelLinkName;

            iDynTree::Rotation relativeRotationZero =
                kinDynComputations->getWorldTransform(parentLinkName).getRotation().inverse()
                * kinDynComputations->getWorldTransform(childLinkName).getRotation();

            iDynTree::Rotation calibrationRotationMeasurementToLink =
                wearableTargets[childTargetName].get()->rotation
                * wearableTargets[childTargetName].get()->rotation * relativeRotationZero;
            wearableTargets[childTargetName].get()->calibrationMeasurementToLink.setRotation(
                calibrationRotationMeasurementToLink);
            yInfo() << LogPrefix << "secondary calibration for " << childTargetName << " is set";
            break;
        }
        case rpcCommand::setRotationOffset: {
            ereaseTargetCalibration(targetName);
            iDynTree::Rotation calibrationRotationMeasurementToLink =
                iDynTree::Rotation::RPY(3.14 * commandPro->roll / 180,
                                        3.14 * commandPro->pitch / 180,
                                        3.14 * commandPro->yaw / 180);
            // add new calibration
            wearableTargets[targetName].get()->calibrationMeasurementToLink.setRotation(
                calibrationRotationMeasurementToLink);
            yInfo() << LogPrefix << "secondary calibration for " << targetName << " is set";
            break;
        }
        default: {
            yWarning() << LogPrefix << "Command not valid";
            return false;
        }
    }

    return true;
}

bool HumanStateProvider::impl::updateWearableTargets()
{
    for (auto wearableTargetEntry : wearableTargets) {
        WearableName wearableName = wearableTargetEntry.second->wearableName;
        hde::TargetName targetName = wearableTargetEntry.first;
        hde::KinematicTargetType targetType = wearableTargetEntry.second->targetType;

        switch (iWear->getSensor(wearableName)->getSensorType()) {
            case sensor::SensorType::VirtualLinkKinSensor: {
                auto sensor = iWear->getVirtualLinkKinSensor(wearableName);
                if (!sensor) {
                    yError() << LogPrefix << "Sensor" << wearableName
                             << "has been added but is not properly configured.";
                    return false;
                }
                if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
                    yWarning() << LogPrefix << "The sensor status of" << wearableName
                               << "is not ok (" << static_cast<double>(sensor->getSensorStatus())
                               << ")";
                    continue;
                }

                wearable::Vector3 position;
                if (!sensor->getLinkPosition(position)) {
                    yWarning() << LogPrefix
                               << "Failed to read link position from virtual link sensor "
                               << wearableName;
                    continue;
                }
                wearableTargetEntry.second->position.setVal(0, position.at(0));
                wearableTargetEntry.second->position.setVal(1, position.at(1));
                wearableTargetEntry.second->position.setVal(2, position.at(2));

                Quaternion orientation;
                if (!sensor->getLinkOrientation(orientation)) {
                    yWarning() << LogPrefix
                               << "Failed to read link orientation from virtual link sensor "
                               << wearableName;
                    continue;
                }
                wearableTargetEntry.second->rotation.fromQuaternion({orientation.data(), 4});

                wearable::Vector3 linearVelocity;
                if (!sensor->getLinkLinearVelocity(linearVelocity)) {
                    yWarning() << LogPrefix
                               << "Failed to read link linear velocity from virtual link sensor "
                               << wearableName;
                    continue;
                }
                wearableTargetEntry.second->linearVelocity.setVal(0, linearVelocity.at(0));
                wearableTargetEntry.second->linearVelocity.setVal(1, linearVelocity.at(1));
                wearableTargetEntry.second->linearVelocity.setVal(2, linearVelocity.at(2));

                wearable::Vector3 angularVelocity;
                if (!sensor->getLinkAngularVelocity(angularVelocity)) {
                    yWarning() << LogPrefix
                               << "Failed to read link angular velocity from virtual link sensor "
                               << wearableName;
                    continue;
                }
                wearableTargetEntry.second->angularVelocity.setVal(0, angularVelocity.at(0));
                wearableTargetEntry.second->angularVelocity.setVal(1, angularVelocity.at(1));
                wearableTargetEntry.second->angularVelocity.setVal(2, angularVelocity.at(2));
                break;
            }
            case ::sensor::SensorType::PoseSensor: {
                auto sensor = iWear->getPoseSensor(wearableName);
                if (!sensor) {
                    yError() << LogPrefix << "Sensor" << wearableName
                             << "has been added but is not properly configured.";
                    return false;
                }
                if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
                    yWarning() << LogPrefix << "The sensor status of" << wearableName
                               << "is not ok (" << static_cast<double>(sensor->getSensorStatus())
                               << ")";
                    continue;
                }

                wearable::Vector3 position;
                if (!sensor->getPosePosition(position)) {
                    yWarning() << LogPrefix
                               << "Failed to read link position from virtual link sensor "
                               << wearableName;
                    continue;
                }
                wearableTargetEntry.second->position.setVal(0, position.at(0));
                wearableTargetEntry.second->position.setVal(1, position.at(1));
                wearableTargetEntry.second->position.setVal(2, position.at(2));

                Quaternion orientation;
                if (!sensor->getPoseOrientationAsQuaternion(orientation)) {
                    yWarning() << LogPrefix
                               << "Failed to read link orientation from virtual link sensor "
                               << wearableName;
                    continue;
                }
                wearableTargetEntry.second->rotation.fromQuaternion({orientation.data(), 4});
                break;
            }
            case ::sensor::SensorType::ForceTorque6DSensor: {
                auto sensor = iWear->getForceTorque6DSensor(wearableName);
                if (!sensor) {
                    yError() << LogPrefix << "Sensor" << wearableName
                             << "has been added but is not properly configured.";
                    return false;
                }
                if (sensor->getSensorStatus() != sensor::SensorStatus::Ok) {
                    yWarning() << LogPrefix << "The sensor status of" << wearableName
                               << "is not ok (" << static_cast<double>(sensor->getSensorStatus())
                               << ")";
                    continue;
                }

                wearable::Vector3 force;
                if (!sensor->getForceTorque3DForce(force)) {
                    yWarning() << LogPrefix << "Failed to read force from forcetorque 6D sensor "
                               << wearableName;
                    continue;
                }

                // If z force is greater then treshold add the contact constraint
                bool contactActive = force.at(2) > wearableTargetEntry.second->contactTreshold;
                // if swtiching to contact, use new position for contact
                if (!wearableTargetEntry.second->contactActive && contactActive) {
                    kinDynComputations->setRobotState(baseTransformSolution,
                                                      jointConfigurationSolution,
                                                      baseVelocitySolution,
                                                      jointVelocitiesSolution,
                                                      worldGravity);
                    auto contactPosition =
                        kinDynComputations
                            ->getWorldTransform(wearableTargetEntry.second->modelLinkName)
                            .getPosition();
                    contactPosition.setVal(2, 0);
                    // recompute the sensor measurement to link offset (sensor_p_link) such that the
                    // target link position is contactPosition: target_position = world_p_measWorld
                    // + world_R_measWorld * (measWorld_p_sensor + measWorld_R_sensor sensor_p_link)
                    wearableTargetEntry.second->calibrationMeasurementToLink.setPosition(
                        ((contactPosition
                          - wearableTargetEntry.second->calibrationWorldToMeasurementWorld
                                .getPosition())
                             .changeCoordinateFrame(
                                 wearableTargetEntry.second->calibrationWorldToMeasurementWorld
                                     .getRotation()
                                     .inverse())
                         - iDynTree::Position(wearableTargetEntry.second->position))
                            .changeCoordinateFrame(wearableTargetEntry.second->rotation.inverse()));
                }

                wearableTargetEntry.second->contactActive = contactActive;
                wearableTargetEntry.second->contactForce = force;

                break;
            }
            default: {
                yError() << LogPrefix << "Sensor Type for taget " << targetName
                         << " can not be used as target.";
                return false;
            }
        }
    }

    return true;
}

bool HumanStateProvider::impl::createLinkPairs()
{
    // Get the model link names according to the modelToWearable link sensor map
    const size_t nrOfSegments = wearableStorage.modelToWearable_LinkName.size();

    segments.resize(nrOfSegments);

    unsigned segmentIndex = 0;
    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        // Get the name of the link from the model and its prefix from iWear
        std::string modelLinkName = humanModel.getLinkName(linkIndex);

        if (wearableStorage.modelToWearable_LinkName.find(modelLinkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        // TODO check if we need this initialization
        // segments[segmentIndex].velocities.zero();

        // Store the name of the link as segment name
        segments[segmentIndex].segmentName = modelLinkName;
        segmentIndex++;
    }

    // Get all the possible pairs composing the model
    std::vector<std::pair<std::string, std::string>> pairNames;
    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>> pairSegmentIndeces;

    // Get the link pair names
    createEndEffectorsPairs(humanModel, segments, pairNames, pairSegmentIndeces);
    linkPairs.reserve(pairNames.size());

    for (unsigned index = 0; index < pairNames.size(); ++index) {
        LinkPairInfo pairInfo;

        pairInfo.parentFrameName = pairNames[index].first;
        pairInfo.parentFrameSegmentsIndex = pairSegmentIndeces[index].first;

        pairInfo.childFrameName = pairNames[index].second;
        pairInfo.childFrameSegmentsIndex = pairSegmentIndeces[index].second;

        yInfo() << "getting the reduced model from: " << pairInfo.parentFrameName << " to "
                << pairInfo.childFrameName;
        // Get the reduced pair model
        if (!getReducedModel(humanModel,
                             pairInfo.parentFrameName,
                             pairInfo.childFrameName,
                             pairInfo.pairModel)) {

            yWarning() << LogPrefix << "failed to get reduced model for the segment pair "
                       << pairInfo.parentFrameName.c_str() << ", "
                       << pairInfo.childFrameName.c_str();
            continue;
        }

        // Move the link pair instance into the vector
        linkPairs.push_back(std::move(pairInfo));
    }

    return true;
}

bool HumanStateProvider::impl::initializePairwisedInverseKinematicsSolver()
{
    for (auto pairInfo = linkPairs.begin(); pairInfo != linkPairs.end(); pairInfo++) {
        // Allocate the ik solver
        pairInfo->ikSolver = std::make_unique<iDynTree::InverseKinematics>();

        // Set ik parameters
        pairInfo->ikSolver->setVerbosity(1);
        pairInfo->ikSolver->setLinearSolverName(linearSolverName);
        pairInfo->ikSolver->setMaxIterations(maxIterationsIK);
        pairInfo->ikSolver->setCostTolerance(costTolerance);
        pairInfo->ikSolver->setDefaultTargetResolutionMode(
            iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
        pairInfo->ikSolver->setRotationParametrization(
            iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

        // Set ik model
        if (!pairInfo->ikSolver->setModel(pairInfo->pairModel)) {
            yWarning() << LogPrefix << "failed to configure IK solver for the segment pair"
                       << pairInfo->parentFrameName.c_str() << ", "
                       << pairInfo->childFrameName.c_str() << " Skipping pair";
            continue;
        }

        // Add parent link as fixed base constraint with identity transform
        pairInfo->ikSolver->addFrameConstraint(pairInfo->parentFrameName,
                                               iDynTree::Transform::Identity());

        // Add child link as a target and set initial transform to be identity
        pairInfo->ikSolver->addTarget(pairInfo->childFrameName, iDynTree::Transform::Identity());

        // Add target position and rotation weights
        pairInfo->positionTargetWeight = posTargetWeight;
        pairInfo->rotationTargetWeight = rotTargetWeight;

        // Add cost regularization term
        pairInfo->costRegularization = costRegularization;

        // Get floating base for the pair model
        pairInfo->floatingBaseIndex = pairInfo->pairModel.getFrameLink(
            pairInfo->pairModel.getFrameIndex(pairInfo->parentFrameName));

        // Set ik floating base
        if (!pairInfo->ikSolver->setFloatingBaseOnFrameNamed(
                pairInfo->pairModel.getLinkName(pairInfo->floatingBaseIndex))) {
            yError() << "Failed to set floating base frame for the segment pair"
                     << pairInfo->parentFrameName.c_str() << ", "
                     << pairInfo->childFrameName.c_str() << " Skipping pair";
            return false;
        }

        // Set initial joint positions size
        pairInfo->sInitial.resize(pairInfo->pairModel.getNrOfJoints());

        // Obtain the joint location index in full model and the lenght of DoFs i.e joints map
        // This information will be used to put the IK solutions together for the full model
        std::vector<std::string> solverJoints;

        // Resize to number of joints in the pair model
        solverJoints.resize(pairInfo->pairModel.getNrOfJoints());

        for (int i = 0; i < pairInfo->pairModel.getNrOfJoints(); i++) {
            solverJoints[i] = pairInfo->pairModel.getJointName(i);
        }

        pairInfo->consideredJointLocations.reserve(solverJoints.size());
        for (auto& jointName : solverJoints) {
            iDynTree::JointIndex jointIndex = humanModel.getJointIndex(jointName);
            if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                yWarning() << LogPrefix << "IK considered joint " << jointName
                           << " not found in the complete model";
                continue;
            }
            iDynTree::IJointConstPtr joint = humanModel.getJoint(jointIndex);

            // Save location index and length of each DoFs
            pairInfo->consideredJointLocations.push_back(
                std::pair<size_t, size_t>(joint->getDOFsOffset(), joint->getNrOfDOFs()));
        }

        // Set the joint configurations size and initialize to zero
        pairInfo->jointConfigurations.resize(solverJoints.size());
        pairInfo->jointConfigurations.zero();

        // Set the joint velocities size and initialize to zero
        pairInfo->jointVelocities.resize(solverJoints.size());
        pairInfo->jointVelocities.zero();

        // Save the indeces
        // TODO: check if link or frame
        pairInfo->parentFrameModelIndex =
            pairInfo->pairModel.getFrameIndex(pairInfo->parentFrameName);
        pairInfo->childFrameModelIndex =
            pairInfo->pairModel.getFrameIndex(pairInfo->childFrameName);

        // Configure KinDynComputation
        pairInfo->kinDynComputations =
            std::unique_ptr<iDynTree::KinDynComputations>(new iDynTree::KinDynComputations());
        pairInfo->kinDynComputations->loadRobotModel(pairInfo->pairModel);

        // Configure relative Jacobian
        pairInfo->relativeJacobian.resize(6, pairInfo->pairModel.getNrOfDOFs());
        pairInfo->relativeJacobian.zero();
    }

    // Initialize IK Worker Pool
    ikPool = std::unique_ptr<IKWorkerPool>(new IKWorkerPool(ikPoolSize, linkPairs, segments));
    if (!ikPool) {
        yError() << LogPrefix << "failed to create IK worker pool";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(mutex);

        // Set the initial solution in the middle between lower and upper limits
        for (auto& linkPair : linkPairs) {
            for (size_t i = 0; i < linkPair.pairModel.getNrOfJoints(); i++) {
                double minJointLimit = linkPair.pairModel.getJoint(i)->getMinPosLimit(i);
                double maxJointLimit = linkPair.pairModel.getJoint(i)->getMaxPosLimit(i);
                double averageJointLimit = (minJointLimit + maxJointLimit) / 2.0;
                linkPair.sInitial.setVal(i, averageJointLimit);
            }
        }
    }

    return true;
}

bool HumanStateProvider::impl::initializeGlobalInverseKinematicsSolver()
{
    // Set global ik parameters
    globalIK.setVerbosity(1);
    globalIK.setLinearSolverName(linearSolverName);
    globalIK.setMaxIterations(maxIterationsIK);
    globalIK.setCostTolerance(costTolerance);
    globalIK.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
    globalIK.setRotationParametrization(
        iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);

    if (!globalIK.setModel(humanModel)) {
        yError() << LogPrefix << "globalIK: failed to load the model";
        return false;
    }

    if (!globalIK.setFloatingBaseOnFrameNamed(floatingBaseFrame)) {
        yError() << LogPrefix << "Failed to set the globalIK floating base frame on link"
                 << floatingBaseFrame;
        return false;
    }

    if (!addInverseKinematicTargets()) {
        yError() << LogPrefix << "Failed to set the globalIK targets";
        return false;
    }

    return true;
}

bool HumanStateProvider::impl::initializeDynamicalInverseKinematicsSolver()
{
    // Set inverse velocity kinematics parameters
    dynamicalInverseKinematics.setInverseVelocityKinematicsResolutionMode(
        inverseVelocityKinematicsSolver);
    dynamicalInverseKinematics.setInverseVelocityKinematicsRegularization(costRegularization);

    if (!dynamicalInverseKinematics.setModel(humanModel)) {
        yError() << LogPrefix << "DynamicalInverseKinematics: failed to load the model";
        return false;
    }

    if (!dynamicalInverseKinematics.setFloatingBaseOnFrameNamed(floatingBaseFrame)) {
        yError() << LogPrefix
                 << "DynamicalInverseKinematics: Failed to set the floating base frame on link"
                 << floatingBaseFrame;
        return false;
    }

    if (!addDynamicalInverseKinematicsTargets()) {
        yError() << LogPrefix << "Failed to set the dynamical inverse velocity kinematics targets";
        return false;
    }

    if (!dynamicalInverseKinematics.setAllJointsVelocityLimit(dynamicalIKJointVelocityLimit)) {
        yError() << LogPrefix << "Failed to set all joints velocity limits";
        return false;
    }

    if (!dynamicalInverseKinematics.setConstraintParametersJointValues(k_u, k_l))
        return false;

    if (custom_jointsVelocityLimitsNames.size() != 0) {
        for (size_t i = 0; i < custom_jointsVelocityLimitsNames.size(); i++) {
            if (!dynamicalInverseKinematics.setJointVelocityLimit(
                    custom_jointsVelocityLimitsIndexes[i], custom_jointsVelocityLimitsValues[i])) {
                yError() << LogPrefix << "Failed to set joint velocity limit for dof " << i;
                return false;
            }
        }
    }

    if (baseVelocityUpperLimit.size() != 0) {
        if (!dynamicalInverseKinematics.setBaseVelocityLimit(baseVelocityLowerLimit,
                                                             baseVelocityUpperLimit)) {
            yError() << LogPrefix << "Failed to set base velocity limit ";
            return false;
        }
    }

    if (customConstraintVariablesIndex.size() != 0) {
        if (!dynamicalInverseKinematics.setLinearJointConfigurationLimits(
                customConstraintVariablesIndex,
                customConstraintUpperBound,
                customConstraintLowerBound,
                customConstraintMatrix))
            return false;
    }

    return true;
}

bool HumanStateProvider::impl::solvePairwisedInverseKinematicsSolver()
{
    {
        std::lock_guard<std::mutex> lock(mutex);

        // Set link segments transformation and velocity
        for (size_t segmentIndex = 0; segmentIndex < segments.size(); segmentIndex++) {

            SegmentInfo& segmentInfo = segments.at(segmentIndex);
            segmentInfo.poseWRTWorld = linkTransformMatrices.at(segmentInfo.segmentName);
            segmentInfo.velocities = linkVelocities.at(segmentInfo.segmentName);
        }
    }

    // Call IK worker pool to solve
    ikPool->runAndWait();

    // Joint link pair ik solutions using joints map from link pairs initialization
    // to solution struct for exposing data through interface
    for (auto& linkPair : linkPairs) {
        size_t jointIndex = 0;
        for (auto& pairJoint : linkPair.consideredJointLocations) {

            // Check if it is a valid 1 DoF joint
            if (pairJoint.second == 1) {
                jointConfigurationSolution.setVal(pairJoint.first,
                                                  linkPair.jointConfigurations.getVal(jointIndex));
                jointVelocitiesSolution.setVal(pairJoint.first,
                                               linkPair.jointVelocities.getVal(jointIndex));

                linkPair.sInitial.setVal(jointIndex,
                                         linkPair.jointConfigurations.getVal(jointIndex));
                jointIndex++;
            }
            else {
                yWarning()
                    << LogPrefix
                    << " Invalid DoFs for the joint, skipping the ik solution for this joint";
                continue;
            }
        }
    }

    return true;
}

bool HumanStateProvider::impl::solveGlobalInverseKinematicsSolver()
{
    // Set global IK initial condition
    if (!globalIK.setFullJointsInitialCondition(&baseTransformSolution,
                                                &jointConfigurationSolution)) {
        yError() << LogPrefix
                 << "Failed to set the joint configuration for initializing the global IK";
        return false;
    }

    // Update ik targets based on wearable input data
    if (!updateInverseKinematicTargets()) {
        yError() << LogPrefix << "Failed to update the targets for the global IK";
        return false;
    }

    // Use a postural task for regularization
    iDynTree::VectorDynSize posturalTaskJointAngles;
    posturalTaskJointAngles.resize(jointConfigurationSolution.size());
    posturalTaskJointAngles.zero();
    if (!globalIK.setDesiredFullJointsConfiguration(posturalTaskJointAngles, costRegularization)) {
        yError() << LogPrefix << "Failed to set the postural configuration of the IK";
        return false;
    }

    if (!globalIK.solve()) {
        yError() << LogPrefix << "Failed to solve global IK";
        return false;
    }

    // Get the global inverse kinematics solution
    globalIK.getFullJointsSolution(baseTransformSolution, jointConfigurationSolution);

    return true;
}

bool HumanStateProvider::impl::solveDynamicalInverseKinematics()
{
    // compute timestep
    double dt;
    if (lastTime < 0.0) {
        dt = period;
    }
    else {
        dt = yarp::os::Time::now() - lastTime;
    };
    lastTime = yarp::os::Time::now();

    for (auto wearableTargetEntry : wearableTargets) {
        hde::KinematicTargetType targetType = wearableTargetEntry.second->targetType;
        ModelLinkName linkName = wearableTargetEntry.second->modelLinkName;
        hde::TargetName targetName = wearableTargetEntry.first;

        switch (targetType) {
            case hde::KinematicTargetType::pose: {
                if (!dynamicalInverseKinematics.updateTargetPose(
                        linkName,
                        wearableTargetEntry.second->getCalibratedPosition(),
                        wearableTargetEntry.second->getCalibratedRotation())) {
                    yError() << LogPrefix << "Failed to update pose target for " << targetName;
                    return false;
                }
                break;
            }
            case hde::KinematicTargetType::poseAndVelocity: {
                if (!dynamicalInverseKinematics.updateTargetPoseAndVelocity(
                        linkName,
                        wearableTargetEntry.second->getCalibratedPosition(),
                        wearableTargetEntry.second->getCalibratedRotation(),
                        wearableTargetEntry.second->getCalibratedLinearVelocity(),
                        wearableTargetEntry.second->getCalibratedAngularVelocity())) {
                    yError() << LogPrefix << "Failed to update pose and velocity target for "
                             << targetName;
                    return false;
                }
                break;
            }
            case hde::KinematicTargetType::position: {
                if (!dynamicalInverseKinematics.updateTargetPosition(
                        linkName, wearableTargetEntry.second->getCalibratedPosition())) {
                    yError() << LogPrefix << "Failed to update position target for " << targetName;
                    return false;
                }
                break;
            }
            case hde::KinematicTargetType::positionAndVelocity: {
                if (!dynamicalInverseKinematics.updateTargetPositionAndVelocity(
                        linkName,
                        wearableTargetEntry.second->getCalibratedPosition(),
                        wearableTargetEntry.second->getCalibratedLinearVelocity())) {
                    yError() << LogPrefix << "Failed to update position and velocity target for "
                             << targetName;
                    return false;
                }
                break;
            }
            case hde::KinematicTargetType::orientation: {
                if (!dynamicalInverseKinematics.updateTargetOrientation(
                        linkName, wearableTargetEntry.second->getCalibratedRotation())) {
                    yError() << LogPrefix << "Failed to update orientation target for "
                             << targetName;
                    return false;
                }
                break;
            }
            case hde::KinematicTargetType::orientationAndVelocity: {
                if (!dynamicalInverseKinematics.updateTargetOrientationAndVelocity(
                        linkName,
                        wearableTargetEntry.second->getCalibratedRotation(),
                        wearableTargetEntry.second->getCalibratedAngularVelocity())) {
                    yError() << LogPrefix << "Failed to update orientation and velocity target for "
                             << targetName;
                    return false;
                }
                break;
            }
            case hde::KinematicTargetType::gravity: {
                if (!dynamicalInverseKinematics.updateTargetOrientation(
                        linkName, wearableTargetEntry.second->getCalibratedRotation())) {
                    yError() << LogPrefix << "Failed to update gravity target for " << targetName;
                    return false;
                }
                break;
            }
            case hde::KinematicTargetType::floorContact: {
                if (!dynamicalInverseKinematics.updateTargetPositionAndVelocity(
                        linkName,
                        wearableTargetEntry.second->getCalibratedPosition(),
                        wearableTargetEntry.second->getCalibratedLinearVelocity())
                    || !dynamicalInverseKinematics.updatePositionTargetAxis(
                        linkName,
                        {wearableTargetEntry.second->contactActive,
                         wearableTargetEntry.second->contactActive,
                         wearableTargetEntry.second->contactActive})) {

                    yError() << LogPrefix << "Failed to update position and velocity target for "
                             << targetName;
                    return false;
                }
                break;
            }
            default: {
                yError() << LogPrefix << "Invalid target type for " << targetName;
                return false;
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutexFlagIntegrator);
        if (!dynamicalInverseKinematics.solve(dt, resetIntegrator))
        {
            return false;
        }
        else 
        {
            resetIntegrator = false;
        }
    }

    dynamicalInverseKinematics.getConfigurationSolution(baseTransformSolution,
                                                        jointConfigurationSolution);
    dynamicalInverseKinematics.getVelocitySolution(baseVelocitySolution, jointVelocitiesSolution);

    return true;
}

bool HumanStateProvider::impl::updateInverseKinematicTargets()
{
    iDynTree::Transform linkTransform;

    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        std::string linkName = humanModel.getLinkName(linkIndex);

        // Skip links with no associated measures (use only links from the configuration)
        if (wearableStorage.modelToWearable_LinkName.find(linkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            continue;
        }

        // For the link used as base insert both the rotation and position cost if not using direcly
        // base measurements and the base is not fixed.
        if (linkName == floatingBaseFrame) {
            if (!(useDirectBaseMeasurement || useFixedBase)) {
                if (!globalIK.updateTarget(
                        linkName, linkTransformMatrices.at(linkName), 1.0, 1.0)) {
                    yError() << LogPrefix << "Failed to update target for floating base"
                             << linkName;
                    return false;
                }
            }
            continue;
        }

        if (linkTransformMatrices.find(linkName) == linkTransformMatrices.end()) {
            yError() << LogPrefix << "Failed to find transformation matrix for link" << linkName;
            return false;
        }

        linkTransform = linkTransformMatrices.at(linkName);
        // if useDirectBaseMeasurement, use the link transform relative to the base
        if (useDirectBaseMeasurement) {
            linkTransform = linkTransformMatrices.at(floatingBaseFrame).inverse() * linkTransform;
        }

        if (!globalIK.updateTarget(linkName, linkTransform, posTargetWeight, rotTargetWeight)) {
            yError() << LogPrefix << "Failed to update target for link" << linkName;
            return false;
        }
    }
    return true;
}

bool HumanStateProvider::impl::addInverseKinematicTargets()
{
    for (auto wearableTargetEntry : wearableTargets) {
        hde::KinematicTargetType targetType = wearableTargetEntry.second->targetType;
        ModelLinkName linkName = wearableTargetEntry.second->modelLinkName;
        hde::TargetName targetName = wearableTargetEntry.first;

        switch (targetType) {
            case hde::KinematicTargetType::pose: {
                if (!globalIK.addTarget(linkName, iDynTree::Transform::Identity(), 1.0, 1.0)) {
                    yError() << LogPrefix << "Failed to taget for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Target " << targetName << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::poseAndVelocity: {
                if (!globalIK.addTarget(linkName, iDynTree::Transform::Identity(), 1.0, 1.0)) {
                    yError() << LogPrefix << "Failed to taget for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Target " << targetName << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::position: {
                if (!globalIK.addPositionTarget(linkName, iDynTree::Transform::Identity(), 1.0)) {
                    yError() << LogPrefix << "Failed to taget for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Target " << targetName << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::positionAndVelocity: {
                if (!globalIK.addPositionTarget(linkName, iDynTree::Transform::Identity(), 1.0)) {
                    yError() << LogPrefix << "Failed to taget for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Target " << targetName << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::orientation: {
                if (!globalIK.addRotationTarget(linkName, iDynTree::Transform::Identity(), 1.0)) {
                    yError() << LogPrefix << "Failed to taget for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Target " << targetName << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::orientationAndVelocity: {
                if (!globalIK.addRotationTarget(linkName, iDynTree::Transform::Identity(), 1.0)) {
                    yError() << LogPrefix << "Failed to taget for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Target " << targetName << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::gravity: {
                yError() << LogPrefix << "Failed to taget for " << targetName
                         << " (gravity taget is not implemented in globalIK)";
                return false;
                break;
            }
            case hde::KinematicTargetType::floorContact: {
                yError() << LogPrefix << "Failed to taget for " << targetName
                         << " (floorContact taget is not implemented in globalIK)";
                return false;
                break;
            }
            default: {
                yError() << LogPrefix << "Invalid target type for " << targetName;
                return false;
            }
        }
    }

    if ((useDirectBaseMeasurement || useFixedBase)
        && !globalIK.addFrameConstraint(floatingBaseFrame, iDynTree::Transform::Identity())) {
        yError() << LogPrefix << "Failed to add constraint for base link" << floatingBaseFrame;
        return false;
    }

    return true;
}

bool HumanStateProvider::impl::addDynamicalInverseKinematicsTargets()
{
    for (auto wearableTargetEntry : wearableTargets) {
        hde::KinematicTargetType targetType = wearableTargetEntry.second->targetType;
        ModelLinkName linkName = wearableTargetEntry.second->modelLinkName;
        hde::TargetName targetName = wearableTargetEntry.first;

        switch (targetType) {
            case hde::KinematicTargetType::pose: {
                if (!dynamicalInverseKinematics.addPoseTarget(linkName,
                                                              wearableTargetEntry.second->position,
                                                              wearableTargetEntry.second->rotation,
                                                              {true, true, true},
                                                              {true, true, true},
                                                              dynamicalIKLinearCorrectionGain,
                                                              dynamicalIKAngularCorrectionGain,
                                                              linVelTargetWeight,
                                                              angVelTargetWeight)) {
                    yError() << LogPrefix << "Failed to add pose target for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Pose Target " << targetName << " added for link "
                        << linkName;
                break;
            }
            case hde::KinematicTargetType::poseAndVelocity: {
                if (!dynamicalInverseKinematics.addPoseAndVelocityTarget(
                        linkName,
                        wearableTargetEntry.second->position,
                        wearableTargetEntry.second->rotation,
                        wearableTargetEntry.second->linearVelocity,
                        wearableTargetEntry.second->angularVelocity,
                        {true, true, true},
                        {true, true, true},
                        dynamicalIKLinearCorrectionGain,
                        dynamicalIKAngularCorrectionGain,
                        dynamicalIKMeasuredLinearVelocityGain,
                        dynamicalIKMeasuredAngularVelocityGain,
                        linVelTargetWeight,
                        angVelTargetWeight)) {
                    yError() << LogPrefix << "Failed to add pose and velocity target for "
                             << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Pose and Velocity Target " << targetName
                        << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::position: {
                if (!dynamicalInverseKinematics.addPositionTarget(
                        linkName,
                        wearableTargetEntry.second->position,
                        {true, true, true},
                        dynamicalIKLinearCorrectionGain,
                        linVelTargetWeight)) {
                    yError() << LogPrefix << "Failed to add position target for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Position Target " << targetName << " added for link "
                        << linkName;
                break;
            }
            case hde::KinematicTargetType::positionAndVelocity: {
                if (!dynamicalInverseKinematics.addPositionAndVelocityTarget(
                        linkName,
                        wearableTargetEntry.second->position,
                        wearableTargetEntry.second->linearVelocity,
                        {true, true, true},
                        dynamicalIKLinearCorrectionGain,
                        dynamicalIKMeasuredLinearVelocityGain,
                        linVelTargetWeight)) {
                    yError() << LogPrefix << "Failed to add position and velocity target for "
                             << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Position and Velocity Target " << targetName
                        << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::orientation: {
                if (!dynamicalInverseKinematics.addOrientationTarget(
                        linkName,
                        wearableTargetEntry.second->rotation,
                        {true, true, true},
                        dynamicalIKAngularCorrectionGain,
                        angVelTargetWeight)) {
                    yError() << LogPrefix << "Failed to add orientation target for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Orientation Target " << targetName << " added for link "
                        << linkName;
                break;
            }
            case hde::KinematicTargetType::orientationAndVelocity: {
                if (!dynamicalInverseKinematics.addOrientationAndVelocityTarget(
                        linkName,
                        wearableTargetEntry.second->rotation,
                        wearableTargetEntry.second->angularVelocity,
                        {true, true, true},
                        dynamicalIKAngularCorrectionGain,
                        dynamicalIKMeasuredAngularVelocityGain,
                        angVelTargetWeight)) {
                    yError() << LogPrefix << "Failed to add orientation and velocity target for "
                             << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Orientation and Velocity Target " << targetName
                        << " added for link " << linkName;
                break;
            }
            case hde::KinematicTargetType::gravity: {
                if (!dynamicalInverseKinematics.addOrientationTarget(
                        linkName,
                        wearableTargetEntry.second->rotation,
                        {false, false, true},
                        dynamicalIKAngularCorrectionGain,
                        angVelTargetWeight)) {
                    yError() << LogPrefix << "Failed to add gravity target for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Gravity Target " << targetName << " added for link "
                        << linkName;
                break;
            }
            case hde::KinematicTargetType::floorContact: {
                if (!dynamicalInverseKinematics.addPositionAndVelocityTarget(
                        linkName,
                        wearableTargetEntry.second->position,
                        wearableTargetEntry.second->linearVelocity,
                        {true, true, true},
                        dynamicalIKLinearCorrectionGain,
                        dynamicalIKMeasuredLinearVelocityGain,
                        linVelTargetWeight)) {
                    yError() << LogPrefix << "Failed to add floorContact target for " << targetName;
                    return false;
                }
                yInfo() << LogPrefix << "Floor Contact Target " << targetName << " added for link "
                        << linkName;
                break;
            }
            default: {
                yError() << LogPrefix << "Invalid target type for " << targetName;
                return false;
            }
        }
    }

    return true;
}

bool HumanStateProvider::impl::computeLinksOrientationErrors(
    std::unordered_map<std::string, iDynTree::Transform> linkDesiredTransforms,
    iDynTree::VectorDynSize jointConfigurations,
    iDynTree::Transform floatingBasePose,
    std::unordered_map<std::string, hde::utils::idyntree::rotation::RotationDistance>&
        linkErrorOrientations)
{
    iDynTree::VectorDynSize zeroJointVelocities = jointConfigurations;
    zeroJointVelocities.zero();

    iDynTree::Twist zeroBaseVelocity;
    zeroBaseVelocity.zero();

    iDynTree::KinDynComputations* computations = kinDynComputations.get();
    computations->setRobotState(
        floatingBasePose, jointConfigurations, zeroBaseVelocity, zeroJointVelocities, worldGravity);

    for (const auto& linkMapEntry : linkDesiredTransforms) {
        const ModelLinkName& linkName = linkMapEntry.first;
        linkErrorOrientations[linkName] = hde::utils::idyntree::rotation::RotationDistance(
            computations->getWorldTransform(linkName).getRotation(),
            linkDesiredTransforms[linkName].getRotation());
    }
    return true;
}

bool HumanStateProvider::impl::computeLinksAngularVelocityErrors(
    std::unordered_map<std::string, iDynTree::Twist> linkDesiredVelocities,
    iDynTree::VectorDynSize jointConfigurations,
    iDynTree::Transform floatingBasePose,
    iDynTree::VectorDynSize jointVelocities,
    iDynTree::Twist baseVelocity,
    std::unordered_map<std::string, iDynTree::Vector3>& linkAngularVelocityError)
{
    iDynTree::KinDynComputations* computations = kinDynComputations.get();
    computations->setRobotState(
        floatingBasePose, jointConfigurations, baseVelocity, jointVelocities, worldGravity);

    for (const auto& linkMapEntry : linkDesiredVelocities) {
        const ModelLinkName& linkName = linkMapEntry.first;
        iDynTree::toEigen(linkAngularVelocityError[linkName]) =
            iDynTree::toEigen(linkDesiredVelocities[linkName].getLinearVec3())
            - iDynTree::toEigen(computations->getFrameVel(linkName).getLinearVec3());
    }

    return true;
}

bool HumanStateProvider::impl::attach(yarp::dev::PolyDriver* poly)
{
    if (!poly) {
        yError() << LogPrefix << "Passed PolyDriver is nullptr";
        return false;
    }

    if (!poly->view(iWear)) {
        yError() << LogPrefix << "Failed to view the IWear interface from the PolyDriver";
        return false;
    }

    while (iWear->getStatus() == WearStatus::WaitingForFirstRead) {
        yInfo() << LogPrefix << "IWear interface waiting for first data. Waiting...";
        yarp::os::Time::delay(5);
    }

    if (iWear->getStatus() != WearStatus::Ok) {
        yError() << LogPrefix << "The status of the attached IWear interface is not ok ("
                 << static_cast<int>(iWear->getStatus()) << ")";
        return false;
    }

    // ======================
    // CHECK WEARABLE TARGETS
    // ======================

    for (auto wearableTargetEntry : wearableTargets) {
        ModelLinkName linkName = wearableTargetEntry.second->modelLinkName;
        hde::TargetName targetName = wearableTargetEntry.first;
        WearableName wearableName = wearableTargetEntry.second->wearableName;

        // Check if the link exist in the model
        if (humanModel.getLinkIndex(linkName) == iDynTree::LINK_INVALID_INDEX) {
            yError() << "Failed to find link " << linkName << " used in target " << targetName;
            return false;
        }

        // Check if the wearable sensor exist and read the type
        auto sensor = iWear->getSensor(wearableName);
        if (!sensor) {
            yError() << "Failed to find sensor " << wearableName << " used in target "
                     << targetName;
            return false;
        }
    }

    // =======================
    // CHECK LINKS AND SENSORS
    // =======================

    // Check that the attached IWear interface contains all the model links
    for (size_t linkIndex = 0; linkIndex < humanModel.getNrOfLinks(); ++linkIndex) {
        // Get the name of the link from the model and its prefix from iWear
        std::string modelLinkName = humanModel.getLinkName(linkIndex);

        if (wearableStorage.modelToWearable_LinkName.find(modelLinkName)
            == wearableStorage.modelToWearable_LinkName.end()) {
            // yWarning() << LogPrefix << "Failed to find" << modelLinkName
            //           << "entry in the configuration map. Skipping this link.";
            continue;
        }

        // Get the name of the sensor associated to the link
        WearableName wearableName = wearableStorage.modelToWearable_LinkName.at(modelLinkName);

        // Try to get the sensor
        auto sensor = iWear->getVirtualLinkKinSensor(wearableName);
        if (!sensor) {
            // yError() << LogPrefix << "Failed to find sensor associated to link" <<
            // wearableName
            //<< "from the IWear interface";
            return false;
        }

        // Create a sensor map entry using the wearable sensor name as key
        wearableStorage.linkSensorsMap[wearableName] = iWear->getVirtualLinkKinSensor(wearableName);
    }

    // ====
    // MISC
    // ====

    yInfo() << LogPrefix << "IWear interface attached successfully";
    return true;
}

void HumanStateProvider::threadRelease()
{
    if (pImpl->ikSolver == SolverIK::pairwised && !pImpl->ikPool->closeIKWorkerPool()) {
        yError() << LogPrefix << "Failed to close the IKWorker pool";
    }
}

bool HumanStateProvider::detachAll()
{
    if (isRunning()) {
        stop();
    }

    pImpl->iWear = nullptr;

    return true;
}

bool HumanStateProvider::attachAll(const yarp::dev::PolyDriverList& driverList)
{
    if (driverList.size() > 1) {
        yError() << LogPrefix << "This wrapper accepts only one attached PolyDriver";
        return false;
    }

    const yarp::dev::PolyDriverDescriptor* driver = driverList[0];

    if (!driver) {
        yError() << LogPrefix << "Passed PolyDriverDescriptor is nullptr";
        return false;
    }

    // attach the device
    if (!pImpl->attach(driver->poly)) {
        return false;
    }

    // Start the PeriodicThread loop
    if (!start()) {
        yError() << LogPrefix << "Failed to start the loop.";
        return false;
    }

    return true;
}

std::vector<std::string> HumanStateProvider::getJointNames() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    std::vector<std::string> jointNames;

    for (size_t jointIndex = 0; jointIndex < pImpl->humanModel.getNrOfJoints(); ++jointIndex) {
        if (pImpl->humanModel.getJoint(jointIndex)->getNrOfDOFs() == 1) {
            jointNames.emplace_back(pImpl->humanModel.getJointName(jointIndex));
        }
    }

    return jointNames;
}

size_t HumanStateProvider::getNumberOfJoints() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->humanModel.getNrOfDOFs();
}

std::string HumanStateProvider::getBaseName() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->floatingBaseFrame;
}

std::vector<double> HumanStateProvider::getJointPositions() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.jointPositions;
}

std::vector<double> HumanStateProvider::getJointVelocities() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.jointVelocities;
}

std::array<double, 6> HumanStateProvider::getBaseVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.baseVelocity;
}

std::array<double, 4> HumanStateProvider::getBaseOrientation() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.baseOrientation;
}

std::array<double, 3> HumanStateProvider::getBasePosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.basePosition;
}
std::array<double, 3> HumanStateProvider::getCoMPosition() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.CoMPosition;
}

std::array<double, 3> HumanStateProvider::getCoMVelocity() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    return pImpl->solution.CoMVelocity;
}

std::vector<hde::TargetName> HumanStateProvider::getAllTargetsName() const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);
    std::vector<std::string> targetsName;
    for (auto wearableTargetEntry : pImpl->wearableTargets) {
        targetsName.push_back(wearableTargetEntry.first);
    }
    return targetsName;
}

std::shared_ptr<hde::WearableSensorTarget>
HumanStateProvider::getTarget(const TargetName name) const
{
    std::lock_guard<std::mutex> lock(pImpl->mutex);

    // TODO: what to do if the target name do not exist
    return pImpl->wearableTargets[name];
}

// This method returns the all link pair names from the full human model
static void createEndEffectorsPairs(
    const iDynTree::Model& model,
    std::vector<SegmentInfo>& humanSegments,
    std::vector<std::pair<std::string, std::string>>& framePairs,
    std::vector<std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>>& framePairIndeces)
{
    // for each element in human segments
    // extract it from the vector (to avoid duplications)
    // Look for it in the model and get neighbours
    std::vector<SegmentInfo> segments(humanSegments);
    size_t segmentCount = segments.size();

    while (!segments.empty()) {
        SegmentInfo segment = segments.back();
        segments.pop_back();
        segmentCount--;

        iDynTree::LinkIndex linkIndex = model.getLinkIndex(segment.segmentName);
        if (linkIndex < 0 || static_cast<unsigned>(linkIndex) >= model.getNrOfLinks()) {
            yWarning("Segment %s not found in the URDF model", segment.segmentName.c_str());
            continue;
        }

        // this for loop should not be necessary, but this can help keeps the backtrace short
        // as we do not assume that we can go back further that this node
        for (unsigned neighbourIndex = 0; neighbourIndex < model.getNrOfNeighbors(linkIndex);
             ++neighbourIndex) {
            // remember the "biforcations"
            std::vector<iDynTree::LinkIndex> backtrace;
            std::vector<iDynTree::LinkIndex>::iterator Iterator_backtrace;

            // and the visited nodes
            std::vector<iDynTree::LinkIndex> visited;

            // I've already visited the starting node
            visited.push_back(linkIndex);
            iDynTree::Neighbor neighbour = model.getNeighbor(linkIndex, neighbourIndex);
            backtrace.push_back(neighbour.neighborLink);

            while (!backtrace.empty()) {
                iDynTree::LinkIndex currentLink = backtrace.back();
                backtrace.pop_back();
                // add the current link to the visited
                visited.push_back(currentLink);

                std::string linkName = model.getLinkName(currentLink);

                // check if this is a human segment
                std::vector<SegmentInfo>::iterator foundSegment =
                    std::find_if(segments.begin(), segments.end(), [&](SegmentInfo& frame) {
                        return frame.segmentName == linkName;
                    });
                if (foundSegment != segments.end()) {
                    std::vector<SegmentInfo>::difference_type foundLinkIndex =
                        std::distance(segments.begin(), foundSegment);
                    // Found! This is a segment
                    framePairs.push_back(
                        std::pair<std::string, std::string>(segment.segmentName, linkName));
                    framePairIndeces.push_back(
                        std::pair<iDynTree::FrameIndex, iDynTree::FrameIndex>(segmentCount,
                                                                              foundLinkIndex));
                    yInfo() << "Segment : " << segment.segmentName
                            << " , associated neighbor : " << linkName
                            << " , found segment: " << foundSegment->segmentName
                            << " , distance: " << foundLinkIndex;
                    break;
                }

                for (unsigned i = 0; i < model.getNrOfNeighbors(currentLink); ++i) {
                    iDynTree::LinkIndex link = model.getNeighbor(currentLink, i).neighborLink;
                    // check if we already visited this segment
                    if (std::find(visited.begin(), visited.end(), link) != visited.end()) {
                        // Yes => skip
                        continue;
                    }
                    Iterator_backtrace = backtrace.begin();
                    backtrace.insert(Iterator_backtrace, link);
                }
            }
        }
    }
}

static bool getReducedModel(const iDynTree::Model& modelInput,
                            const std::string& parentFrame,
                            const std::string& endEffectorFrame,
                            iDynTree::Model& modelOutput)
{
    iDynTree::FrameIndex parentFrameIndex;
    iDynTree::FrameIndex endEffectorFrameIndex;
    std::vector<std::string> consideredJoints;
    iDynTree::Traversal traversal;
    iDynTree::LinkIndex parentLinkIdx;
    iDynTree::IJointConstPtr joint;
    iDynTree::ModelLoader loader;

    // Get frame indices
    parentFrameIndex = modelInput.getFrameIndex(parentFrame);
    endEffectorFrameIndex = modelInput.getFrameIndex(endEffectorFrame);

    if (parentFrameIndex == iDynTree::FRAME_INVALID_INDEX) {
        yError() << LogPrefix << " Invalid parent frame: " << parentFrame;
        return false;
    }
    else if (endEffectorFrameIndex == iDynTree::FRAME_INVALID_INDEX) {
        yError() << LogPrefix << " Invalid End Effector Frame: " << endEffectorFrame;
        return false;
    }

    // Select joint from traversal
    modelInput.computeFullTreeTraversal(traversal, modelInput.getFrameLink(parentFrameIndex));

    iDynTree::LinkIndex visitedLink = modelInput.getFrameLink(endEffectorFrameIndex);

    while (visitedLink != traversal.getBaseLink()->getIndex()) {
        parentLinkIdx = traversal.getParentLinkFromLinkIndex(visitedLink)->getIndex();
        joint = traversal.getParentJointFromLinkIndex(visitedLink);

        // Check if the joint is supported
        if (modelInput.getJoint(joint->getIndex())->getNrOfDOFs() == 1) {
            consideredJoints.insert(consideredJoints.begin(),
                                    modelInput.getJointName(joint->getIndex()));
        }
        else {
            yWarning() << LogPrefix << "Joint " << modelInput.getJointName(joint->getIndex())
                       << " is ignored as it has ("
                       << modelInput.getJoint(joint->getIndex())->getNrOfDOFs() << " DOFs)";
        }

        visitedLink = parentLinkIdx;
    }

    if (!loader.loadReducedModelFromFullModel(modelInput, consideredJoints)) {
        yWarning() << LogPrefix << " failed to select joints: ";
        for (std::vector<std::string>::const_iterator i = consideredJoints.begin();
             i != consideredJoints.end();
             ++i) {
            yWarning() << *i << ' ';
        }
        return false;
    }

    modelOutput = loader.model();

    return true;
}
