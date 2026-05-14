// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <hde/interfaces/IHumanState.h>
#include <hde/interfaces/IHumanWrench.h>
#include <hde/interfaces/IWearableTargets.h>
#include <hde/interfaces/IHumanDynamics.h>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <iDynTree/ModelLoader.h>
#include <iDynTree/Visualizer.h>
#include <iDynTree/EigenHelpers.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <thread>
#include <csignal>
#include <unordered_map>

const std::string ModuleName = "HumanStateVisualizer";
const std::string LogPrefix = ModuleName + " :";

std::atomic<bool> isClosing{false};

struct JointEffortData
{
    std::string parentLinkName;
    std::string sphericalJointName;
    std::vector<iDynTree::JointIndex> fakeJointsIndices;
    double effortMax;
    double effort;
};

void my_handler(int signal)
{
    isClosing = true;
}

#ifdef WIN32

#include <windows.h>

BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType) {
        // Handle the CTRL-C signal.
    case CTRL_C_EVENT:
    case CTRL_CLOSE_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        my_handler(0);
        return TRUE;

    // Handle all other events
    default:
        return FALSE;
    }
}
#endif

void handleSigInt()
{
#ifdef WIN32
    SetConsoleCtrlHandler(CtrlHandler, TRUE);
#else
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = &my_handler;
    sigaction(SIGINT, &action, NULL);
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGABRT, &action, NULL);
#endif
}

int main(int argc, char* argv[])
{

    // Listen to signals for closing in a clean way the application
    handleSigInt();

    // parse the configuraiton options
    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("HumanStateVisualizer.ini");
    rf.configure(argc, argv);
    
    if (rf.isNull())
    {
        yError() << LogPrefix << "Empty configuration file.";
        return EXIT_FAILURE;
    }

    if( !(rf.check("modelURDFName") && rf.find("modelURDFName").isString()) ) 
    {
        yError() << LogPrefix << "'modelURDFName' option not found or not valid.";
        return EXIT_FAILURE;
    }
    std::string urdfFile = rf.find("modelURDFName").asString();

    bool ignoreMissingLinks;
    if( !(rf.check("ignoreMissingLinks") && rf.find("ignoreMissingLinks").isBool()) ) 
    {
        yWarning() << LogPrefix << "'ignoreMissingLinks' option not found or not valid. It is set to False.";
        ignoreMissingLinks = false;
    }
    else
    {
        ignoreMissingLinks = rf.find("ignoreMissingLinks").asBool();
    }

    bool visualizeWrenches;
    if( !(rf.check("visualizeWrenches") && rf.find("visualizeWrenches").isBool()) ) 
    {
        yWarning() << LogPrefix << "'visualizeWrenches' option not found or not valid. It is set to False.";
        visualizeWrenches = false;
    }
    else
    {
        visualizeWrenches = rf.find("visualizeWrenches").asBool();
    }

    bool visualizeFrames;
    if( !(rf.check("visualizeFrames") && rf.find("visualizeFrames").isBool()) ) 
    {
        yWarning() << LogPrefix << "'visualizeFrames' option not found or not valid. It is set to False.";
        visualizeFrames = false;
    }
    visualizeFrames = rf.find("visualizeFrames").asBool();

    bool visualizeTargets;
    if( !(rf.check("visualizeTargets") && rf.find("visualizeTargets").isBool()) ) 
    {
        yWarning() << LogPrefix << "'visualizeTargets' option not found or not valid. It is set to False.";
        visualizeTargets = false;
    }
    visualizeTargets = rf.find("visualizeTargets").asBool();

    bool visualizeEfforts;
    if (!(rf.check("visualizeEfforts") && rf.find("visualizeEfforts").isBool()) )
    {
        yWarning() << LogPrefix
                   << "'visualizeEfforts' option not found or not valid. It is set to False.";
        visualizeEfforts = false;
    }
    visualizeEfforts = rf.find("visualizeEfforts").asBool();

    bool setBackgroundColor = true;
    iDynTree::Vector4 backgroundColorVector;
    if (!(rf.check("colorBackground") && rf.find("colorBackground").isList()
          && rf.find("colorBackground").asList()->size() == 4))
    {
        yError() << LogPrefix << "'colorBackground' option not found or not valid. Setting default background.";
        setBackgroundColor = false;
    }
    if (setBackgroundColor)
    {
        for (size_t idx = 0; idx < 4; idx++) {
            if (!(rf.find("colorBackground").asList()->get(idx).isFloat64())) {
                yError() << LogPrefix << "'colorBackground' entry [ " << idx
                         << " ] is not valid. Setting default background.";
                setBackgroundColor = false;
            }
            else
            {
                backgroundColorVector[idx] = rf.find("colorBackground").asList()->get(idx).asFloat64();

            }
        }
    }

    iDynTree::Position cameraDeltaPosition;
    if( !(rf.check("cameraDeltaPosition") && rf.find("cameraDeltaPosition").isList() && rf.find("cameraDeltaPosition").asList()->size() == 3) ) 
    {
        yError() << LogPrefix << "'cameraDeltaPosition' option not found or not valid.";
        return EXIT_FAILURE;
    }
    for (size_t idx = 0; idx < 3; idx++)
    {
        if ( !(rf.find("cameraDeltaPosition").asList()->get(idx).isFloat64()) )
        {
            yError() << LogPrefix << "'cameraDeltaPosition' entry [ " << idx << " ] is not valid.";
            return EXIT_FAILURE;
        }
        cameraDeltaPosition.setVal(idx, rf.find("cameraDeltaPosition").asList()->get(idx).asFloat64());
    }

    if( !(rf.check("useFixedCamera") && rf.find("useFixedCamera").isBool()) ) 
    {
        yError() << LogPrefix << "'useFixedCamera' option not found or not valid.";
        return EXIT_FAILURE;
    }
    bool useFixedCamera = rf.find("useFixedCamera").asBool();

    iDynTree::Position fixedCameraTarget = iDynTree::Position(0, 0, 0);
    if (useFixedCamera) 
    {
        if( !(rf.check("fixedCameraTarget") && rf.find("fixedCameraTarget").isList() && rf.find("fixedCameraTarget").asList()->size() == 3) ) 
        {
            yError() << LogPrefix << "'fixedCameraTarget' option not found or not valid.";
            return EXIT_FAILURE;
        }
        for (size_t idx = 0; idx < 3; idx++)
        {
            if ( !(rf.find("fixedCameraTarget").asList()->get(idx).isFloat64()) )
            {
                yError() << LogPrefix << "'fixedCameraTarget' entry [ " << idx << " ] is not valid.";
                return EXIT_FAILURE;
            }
            fixedCameraTarget.setVal(idx, rf.find("fixedCameraTarget").asList()->get(idx).asFloat64());
        }
    }

    if( !(rf.check("maxVisualizationFPS") && rf.find("maxVisualizationFPS").isInt32() && rf.find("maxVisualizationFPS").asInt32() > 0) ) 
    {
        yError() << LogPrefix << "'maxVisualizationFPS' option not found or not valid.";
        return EXIT_FAILURE;
    }
    unsigned int maxVisualizationFPS = rf.find("maxVisualizationFPS").asInt32();

    if( !(rf.check("humanStateDataPortName") && rf.find("humanStateDataPortName").isString()) ) 
    {
        yError() << LogPrefix << "'humanStateDataPortName' option not found or not valid.";
        return EXIT_FAILURE;
    }
    std::string humanStateDataPortName = rf.find("humanStateDataPortName").asString();

    // Visualize Wrenches Options
    std::string humanWrenchServerPortName;
    std::vector<std::string> wrenchSourceLinks;
    if (visualizeWrenches)
    {
        if( !(rf.check("humanWrenchServerPortName") && rf.find("humanWrenchServerPortName").isString()) )
        {
            yError() << LogPrefix << "'humanWrenchServerPortName' option not found or not valid. Wrench Visualization will be disabled";
            visualizeWrenches = false;
        }
        else
        {
            humanWrenchServerPortName = rf.find("humanWrenchServerPortName").asString();
        }
    }

    if (visualizeWrenches)
    {
        if( !(rf.check("wrenchSourceLinks") && rf.find("wrenchSourceLinks").isList()) )
        {
            yError() << LogPrefix << "'wrenchSourceLinks' option not found or not valid. Wrench Visualization will be disabled";
            visualizeWrenches = false;
        }
        else
        {
            auto wrenchSourceLinksList = rf.find("wrenchSourceLinks").asList();
            for (size_t it = 0; it < wrenchSourceLinksList->size(); it++)
            {
                if(!wrenchSourceLinksList->get(it).isString())
                {
                    yError() << LogPrefix << "in 'wrenchSourceLinks' there is a field that is not a string.";
                    return EXIT_FAILURE;
                }
                wrenchSourceLinks.push_back(wrenchSourceLinksList->get(it).asString());
            }
        }
    }
    int numberOfWrenchElements = 6 * wrenchSourceLinks.size();

    double forceScalingFactor;
    if (visualizeWrenches)
    {
        if ( !(rf.check("forceScalingFactor") && rf.find("forceScalingFactor").isFloat64()) )
        {
            yError() << LogPrefix << "'forceScalingFactor' option not found or not valid.";
            return EXIT_FAILURE;
        }
        forceScalingFactor = rf.find("forceScalingFactor").asFloat64();
    }

    // Visualzie Efforts Options
    std::string humanDynamicsDataPortName;
    std::vector<JointEffortData> modelEffortData;

    if (visualizeEfforts) {
        if (!(rf.check("humanDynamicsServerPortName")
              && rf.find("humanDynamicsServerPortName").isString())) {
            yError() << LogPrefix
                     << "'humanDynamicsServerPortName' option not found or not valid. Effort Visualization will be disabled";
            visualizeEfforts = false;
        }
        else {
            humanDynamicsDataPortName = rf.find("humanDynamicsServerPortName").asString();
        }
    }
    if (visualizeEfforts) {
        if (!(rf.check("parentLinkNames") && rf.find("parentLinkNames").isList())) {
            yError() << LogPrefix
                     << "'parentLinkNames' option not found or valid. Efforts Visualization will "
                        "be disabled";
            visualizeEfforts = false;
        }
        else {
            auto parentLinkNamesList = rf.find("parentLinkNames").asList();
            for (size_t it = 0; it < parentLinkNamesList->size(); it++) {
                if (!parentLinkNamesList->get(it).isString()) {
                    yError() << LogPrefix
                             << "in 'parentLinkNames' there is a field that is not a string.";
                    return EXIT_FAILURE;
                }
                modelEffortData.emplace_back();
                modelEffortData.back().parentLinkName = parentLinkNamesList->get(it).asString();
            }
        }
    }
    if (visualizeEfforts) 
    {
        if (!(rf.check("sphericalJointNames") && rf.find("sphericalJointNames").isList()) )
        {
            yError() << LogPrefix
                     << "'sphericalJointNames' option not found or valid. Efforts Visualization will "
                        "be disabled";
            visualizeEfforts = false;
        }
        else
        {
            auto sphericalJointNamesList = rf.find("sphericalJointNames").asList();
            for (size_t it = 0; it < sphericalJointNamesList->size(); it++) {
                if (!sphericalJointNamesList->get(it).isString()) {
                    yError() << LogPrefix
                             << "in 'sphericalJointNames' there is a field that is not a string.";
                    return EXIT_FAILURE;
                }
                modelEffortData[it].sphericalJointName = sphericalJointNamesList->get(it).asString();
            }
        }
        
    }
    if (visualizeEfforts)
    {
        if (!(rf.check("maxEffort") && rf.find("maxEffort").isList())) {
            yError()
                << LogPrefix
                << "'maxEffort' option not found or valid. Efforts Visualization will "
                   "be disabled";
            visualizeEfforts = false;
        }
        else {
            auto maxEffortList = rf.find("maxEffort").asList();
            for (size_t it = 0; it < maxEffortList->size(); it++) {
                if (!maxEffortList->get(it).isFloat64()) {
                    yError() << LogPrefix
                             << "in 'maxEffort' there is a field that is not a number.";
                    return EXIT_FAILURE;
                }
                modelEffortData[it].effortMax = maxEffortList->get(it).asFloat64();
            }
        }
    }


    // Visualize Frames Options
    std::vector<std::string> visualizedLinksFrame;
    if (visualizeFrames)
    {
        if( !(rf.check("visualizedLinksFrame") && rf.find("visualizedLinksFrame").isList()) )
        {
            yError() << LogPrefix << "'visualizedLinksFrame' option not found or not valid. Frames Visualization will be disabled";
            visualizeFrames = false;
        }
        else
        {
            auto visualizedLinksFrameList = rf.find("visualizedLinksFrame").asList();
            for (size_t it = 0; it < visualizedLinksFrameList->size(); it++)
            {
                if(!visualizedLinksFrameList->get(it).isString())
                {
                    yError() << LogPrefix << "in 'visualizedLinksFrame' there is a field that is not a string.";
                    return EXIT_FAILURE;
                }
                visualizedLinksFrame.push_back(visualizedLinksFrameList->get(it).asString());
            }
        }
    }

    double linksFrameScalingFactor = 1.0;
    if (visualizeFrames)
    {
        if ( !(rf.check("linksFrameScalingFactor") && rf.find("linksFrameScalingFactor").isFloat64()) )
        {
            yWarning() << LogPrefix << "'linksFrameScalingFactor' option not found or not valid. Using default scale factor = 1.0";
        }
        else
        {
            linksFrameScalingFactor = rf.find("linksFrameScalingFactor").asFloat64();
        }
    }

    // Visualize Targets Options
    std::vector<hde::TargetName> visualizedTargetsFrame;
    if (visualizeTargets)
    {
        if( !(rf.check("visualizedTargetsFrame") && rf.find("visualizedTargetsFrame").isList()) )
        {
            yError() << LogPrefix << "'visualizedTargetsFrame' option not found or not valid. Frames Visualization will be disabled";
            visualizeTargets = false;
        }
        else
        {
            auto visualizedTargetsFrameList = rf.find("visualizedTargetsFrame").asList();
            for (size_t it = 0; it < visualizedTargetsFrameList->size(); it++)
            {
                if(!visualizedTargetsFrameList->get(it).isString())
                {
                    yError() << LogPrefix << "in 'visualizedTargetsFrame' there is a field that is not a string.";
                    return EXIT_FAILURE;
                }
                visualizedTargetsFrame.push_back(visualizedTargetsFrameList->get(it).asString());
            }
        }
    }

    double targetsFrameScalingFactor = 1.0;
    double targetsForceScalingFactor = 1.0;
    if (visualizeTargets)
    {
        if ( !(rf.check("targetsFrameScalingFactor") && rf.find("targetsFrameScalingFactor").isFloat64()) )
        {
            yWarning() << LogPrefix << "'targetsFrameScalingFactor' option not found or not valid. Using default scale factor = 1.0";
        }
        else if ( !(rf.check("targetsForceScalingFactor") && rf.find("targetsForceScalingFactor").isFloat64()) )
        {
            yWarning() << LogPrefix << "'targetsForceScalingFactor' option not found or not valid. Using default scale factor = 0.1";
        }
        else
        {
            targetsFrameScalingFactor = rf.find("targetsFrameScalingFactor").asFloat64();
            targetsForceScalingFactor = rf.find("targetsForceScalingFactor").asFloat64();
        }
    }

    std::string wearableTargetsServerPortName;
    if (visualizeTargets)
    {
        if( !(rf.check("wearableTargetsServerPortName") && rf.find("wearableTargetsServerPortName").isString()) )
        {
            yError() << LogPrefix << "'wearableTargetsServerPortName' option not found or not valid. Wrench Visualization will be disabled";
            visualizeTargets = false;
        }
        else
        {
            wearableTargetsServerPortName = rf.find("wearableTargetsServerPortName").asString();
        }
    }


    // load model
    std::string urdfFilePath = rf.findFile(urdfFile);
    if (urdfFilePath.empty()) {
        yError() << LogPrefix << "Failed to find file" << urdfFile;
        return EXIT_FAILURE;
    }

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(urdfFilePath) || !modelLoader.isValid()) {
        yError() << LogPrefix << "Failed to load model" << urdfFilePath;
        return EXIT_FAILURE;
    }

    iDynTree::Model model = modelLoader.model();

    // check if wrench sources are found in the model and save the link index
    std::vector<iDynTree::LinkIndex> wrenchSourceLinkIndices;
    if (visualizeWrenches)
    {
        for (auto wrenchSourceLink : wrenchSourceLinks)
        {
            auto frameIndex = model.getLinkIndex(wrenchSourceLink);
            if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
            {
                yError() << LogPrefix << "wrench source link [ " << wrenchSourceLink << " ] not found in the visualized model";
                return EXIT_FAILURE;
            }
            wrenchSourceLinkIndices.push_back(frameIndex);
        }
    }

    // check if links are found in the model and save the link index
    std::vector<iDynTree::LinkIndex> visualizedLinkIndices;
    if (visualizeFrames)
    {
        for (auto visualizedLink : visualizedLinksFrame)
        {
            auto frameIndex = model.getLinkIndex(visualizedLink);
            if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
            {
                yError() << LogPrefix << "visualized link [ " << visualizedLink << " ] not found in the visualized model";
                return EXIT_FAILURE;
            }
            visualizedLinkIndices.push_back(frameIndex);
        }
    }

    // initialise yarp network
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<< LogPrefix <<"[main] Unable to find YARP network";
        return EXIT_FAILURE;
    }


    // initialize iHumanState interface from client
    yarp::dev::PolyDriver humanStateClientDevice;
    hde::interfaces::IHumanState* iHumanState{nullptr};

    yarp::os::Property clientOptions;
    clientOptions.put("device", "human_state_nwc_yarp");
    clientOptions.put("humanStateDataPort", humanStateDataPortName);

    if(!humanStateClientDevice.open(clientOptions))
    {
        yError() << LogPrefix << "Failed to connect client device (iHumanState)";
        return EXIT_FAILURE;
    }
    if(!humanStateClientDevice.view(iHumanState) || !iHumanState )
    {
        yError() << LogPrefix << "Failed to view iHumanState interface";
        return EXIT_FAILURE;
    }

    // wait for the iHumanState to be initialized
    while (iHumanState->getBaseName().empty())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        yInfo() << LogPrefix << "Waiting for data from HumanStateClient";
    }
     
    // compare the iHumanState base and joint names with the visualization model
    yInfo() << LogPrefix << "Human State Interface providing data for the base link [ " << iHumanState->getBaseName() << " ]";
    if ( iHumanState->getBaseName() != model.getLinkName(model.getDefaultBaseLink()))
    {
        model.setDefaultBaseLink(model.getLinkIndex(iHumanState->getBaseName()));
        yInfo() << LogPrefix << "Default base link of the visualized model is changed to " << iHumanState->getBaseName();
    }
    yInfo() << LogPrefix << "Human State Interface providing data from [ " << iHumanState->getJointNames().size() << " ] joints";
    
    for (auto jointName : iHumanState->getJointNames())
    {
        if (model.getJointIndex(jointName) == iDynTree::JOINT_INVALID_INDEX)
        {
            if (!ignoreMissingLinks)
            {
                yError() << LogPrefix << "joint [ " << jointName << " ] not found in the visualized model." 
                         << " Set ignoreMissingLinks true if you want to ignore the missing joints.";
                return EXIT_FAILURE;
            }
            yWarning() << LogPrefix << "joint [ " << jointName << " ] not found in the visualized model, "
                       << "the joint will be ignored.";
        }
    }

    // initialize iWearableTargets interface from client
    std::unordered_map<hde::TargetName, std::shared_ptr<hde::WearableSensorTarget>> targets;
    yarp::dev::PolyDriver wearableTargetsClientDevice;
    if (visualizeTargets) {
        hde::interfaces::IWearableTargets* iWearableTargets{nullptr};

        yarp::os::Property wearableTargetsClientOptions;
        wearableTargetsClientOptions.put("device", "wearable_targets_nwc_yarp");
        wearableTargetsClientOptions.put("wearableTargetsDataPort", wearableTargetsServerPortName);
        std::cerr << "my server port name is: "  << wearableTargetsServerPortName << std::endl;

        if(!wearableTargetsClientDevice.open(wearableTargetsClientOptions))
        {
            yError() << LogPrefix << "Failed to connect client device (iWearableTargets)";
            return EXIT_FAILURE;
        }
        if(!wearableTargetsClientDevice.view(iWearableTargets) || !iWearableTargets )
        {
            yError() << LogPrefix << "Failed to view iWearableTargets interface";
            return EXIT_FAILURE;
        }

        // wait for the iWearableTargets to be initialized
        std::this_thread::sleep_for(std::chrono::seconds(1));
        while (iWearableTargets->getAllTargetsName().empty())
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            yInfo() << LogPrefix << "Waiting for data from WearableTargetsClient";
        }

        // Verify the selected targets exist, and that the associated link exists in the visualized model
        // and create a vector with the targets
        auto targetsName = iWearableTargets->getAllTargetsName();
        for (auto visualizedTargetFrame : visualizedTargetsFrame)
        {
            auto target = iWearableTargets->getTarget(visualizedTargetFrame);

            // check if target exists
            if (!target)
            {
                yError() << LogPrefix << "target [ " << visualizedTargetFrame << " ] not found in iWearableTargets." ;
                return EXIT_FAILURE;
            }

            // check if frame exists
            auto frameIndex = model.getLinkIndex(target.get()->modelLinkName);
            if (frameIndex == iDynTree::FRAME_INVALID_INDEX)
            {
                yError() << LogPrefix << "target link [ " << target.get()->modelLinkName << " ] not found in the visualized model";
                return EXIT_FAILURE;
            }

            // add the target to the vector
            targets.emplace(visualizedTargetFrame, target);
        }
    }

    // initialize iHumanWrench 
    yarp::dev::PolyDriver humanWrenchClientDevice;
    hde::interfaces::IHumanWrench* iHumanWrench{nullptr};
    if (visualizeWrenches)
    {
        yarp::os::Property humanWrenchClientOptions;
        humanWrenchClientOptions.put("device", "human_wrench_nwc_yarp");
        humanWrenchClientOptions.put("humanWrenchDataPort", humanWrenchServerPortName);

        if(!humanWrenchClientDevice.open(humanWrenchClientOptions))
        {
            yError() << LogPrefix << "Failed to connect client device (iHumanWrench)";
            return EXIT_FAILURE;
        }
        if(!humanWrenchClientDevice.view(iHumanWrench) || !iHumanWrench )
        {
            yError() << LogPrefix << "Failed to view iHumanWrench interface";
            return EXIT_FAILURE;
        }

        // wait for the iHumanWrench to be initialized
        while (iHumanWrench->getWrenchSourceNames().empty())
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            yInfo() << LogPrefix << "Waiting for data from HumanWrenchClient";
        }

        if(iHumanWrench->getNumberOfWrenchSources() != wrenchSourceLinks.size() )
        {
            yError() << LogPrefix << "expected " << wrenchSourceLinks.size() << " wrench sources in port " << humanWrenchServerPortName
                    << ", received " << iHumanWrench->getNumberOfWrenchSources();
            return EXIT_FAILURE;
        }
    }

    // initialize iHumanDynamics
    yarp::dev::PolyDriver humanDynamicsClientDevice;
    hde::interfaces::IHumanDynamics* iHumanDynamics{nullptr};
    std::vector<double> jointTorques;

    if (visualizeEfforts)
    {
        yarp::os::Property humanDynamicsClientOptions;
        humanDynamicsClientOptions.put("device", "human_dynamics_nwc_yarp");
        humanDynamicsClientOptions.put("humanDynamicsDataPort", humanDynamicsDataPortName);

        if (!humanDynamicsClientDevice.open(humanDynamicsClientOptions))
        {
            yError() << LogPrefix << "Failed to connect client device (iHumanDynamics)";
            return EXIT_FAILURE;
        }
        if (!humanDynamicsClientDevice.view(iHumanDynamics) || !iHumanDynamics)
        {
            yError() << LogPrefix << "Failed to view iHumanDynamics interface";
            return EXIT_FAILURE;
        }

         // wait for the iHumanDynamics to be initialized
        std::this_thread::sleep_for(std::chrono::seconds(1));
        while (iHumanDynamics->getJointNames().empty()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            yInfo() << LogPrefix << "Waiting for data from humanDynamicsClient";
        }

        // Get the fake joint indices
        std::vector<std::string> URDFjoints = iHumanDynamics->getJointNames();

        // Check all the occurences of sphericalJointName* in the urdf model joints 
        for (unsigned jointIdx = 0; jointIdx < URDFjoints.size(); ++jointIdx) {
            // Name of the processed urdf joint
            const std::string urdfJointName = URDFjoints[jointIdx];
            // Find if one of the sphericalJointNames from the conf is a substring
            for (auto& effortData : modelEffortData) {
                if (urdfJointName.find(effortData.sphericalJointName) != std::string::npos) {// (sphericalJointName FROM CONFIG FILE)
                    // Store the index
                    effortData.fakeJointsIndices.push_back(jointIdx);
                    yInfo() << "Adding" << urdfJointName << "to" << effortData.parentLinkName << "with" << effortData.sphericalJointName;
                }
            }
        }
       
    }

    // initialize state variables for visualization
    std::array<double, 3> basePositionInterface;
    std::array<double, 4> baseOrientationInterface;
    std::vector<double> jointPositionsInterface;
    std::vector<std::string> jointNames;
    iDynTree::Vector4 baseOrientationQuaternion;
    iDynTree::Position basePosition;
    iDynTree::Position basePositionOld = fixedCameraTarget;
    iDynTree::Transform linkTransform;
    iDynTree::Vector3 gravityVector;
    iDynTree::Vector3 contactForceVector;
    bool contactActive;
    iDynTree::Direction force;

    iDynTree::Transform wHb = iDynTree::Transform::Identity();
    iDynTree::VectorDynSize joints(model.getNrOfDOFs());
    joints.zero();

    // initialize visualization
    iDynTree::Visualizer viz;
    iDynTree::VisualizerOptions options;

    options.winHeight = rf.check("windowHeight", yarp::os::Value(options.winHeight)).asInt32();
    options.winWidth = rf.check("windowWidth", yarp::os::Value(options.winWidth)).asInt32();

    viz.init(options);

    viz.camera().setPosition(cameraDeltaPosition);
    viz.camera().setTarget(fixedCameraTarget);
    
    viz.setColorPalette("meshcat");

    viz.camera().animator()->enableMouseControl(true);
    
    viz.addModel(model, "human");

    if (setBackgroundColor)
    {
        iDynTree::ColorViz colorBackground(backgroundColorVector);
        viz.environment().setBackgroundColor(colorBackground);
    }

    if (visualizeWrenches)
    {
        for (size_t vectorIndex = 0; vectorIndex < wrenchSourceLinks.size(); vectorIndex++)
        {
            linkTransform = viz.modelViz("human").getWorldLinkTransform(wrenchSourceLinkIndices.at(vectorIndex));
            for (size_t i = 0; i < 3; i++)
            {
                force.setVal(i, forceScalingFactor * iHumanWrench->getWrenches()[6 * vectorIndex + i]);
            }
            force = linkTransform.getRotation() * force;
            viz.vectors().addVector(linkTransform.getPosition(), force);
        }
    }

    if (visualizeFrames)
    {
        for (size_t vectorIndex = 0; vectorIndex < visualizedLinksFrame.size(); vectorIndex++)
        {
            linkTransform = viz.modelViz("human").getWorldLinkTransform(visualizedLinksFrame.at(vectorIndex));
            viz.frames().addFrame(linkTransform, linksFrameScalingFactor);
        }
    }

    if (visualizeTargets)
    {
        for (auto targetEntity : targets)
        {
            if ( targetEntity.second.get()->targetType == hde::KinematicTargetType::pose ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::poseAndVelocity ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::position ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::positionAndVelocity ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::orientation ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::orientationAndVelocity ||
                 targetEntity.second.get()->targetType == hde::KinematicTargetType::floorContact)
            {
                linkTransform = iDynTree::Transform(targetEntity.second.get()->getCalibratedRotation(), iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                viz.frames().addFrame(linkTransform, targetsFrameScalingFactor);
            }
            if ( targetEntity.second.get()->targetType == hde::KinematicTargetType::gravity )
            {
                linkTransform = iDynTree::Transform(targetEntity.second.get()->getCalibratedRotation(), iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                iDynTree::toEigen(gravityVector) = targetsFrameScalingFactor * iDynTree::toEigen(linkTransform.getRotation()).row(2);
                viz.vectors().addVector(linkTransform.getPosition(), gravityVector );
            }
if ( targetEntity.second.get()->targetType == hde::KinematicTargetType::floorContact )
            {
                linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                iDynTree::toEigen(contactForceVector) = targetsForceScalingFactor *  iDynTree::toEigen(targetEntity.second.get()->contactForce);
                viz.vectors().addVector(linkTransform.getPosition(), contactForceVector);
            }            
        }
    }

    if (visualizeEfforts)
    {
        iDynTree::ColorViz modelColor(0.0, 0.0, 0.0, 0.3);
        viz.modelViz("human").setModelColor(modelColor);

        iDynTree::Sphere sphere;
        sphere.setRadius(0.08);
        iDynTree::ColorViz color(0.0, 0.0, 0.0, 1);
        iDynTree::Material material = sphere.getMaterial();
        material.setColor(color.toVector4());
        sphere.setMaterial(material);

        for (auto& jointEffortData : modelEffortData)
        {
            viz.shapes().addShape(sphere, "human", jointEffortData.parentLinkName);
        }
    }

    // start Visualization
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point lastViz = std::chrono::steady_clock::now();

    long minimumMicroSecViz = std::round(1e6 / (double) maxVisualizationFPS);

    std::cerr << "RUNNING VIZ!" << std::endl;

    while(viz.run() && !isClosing)
    {
        now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::microseconds>(now - lastViz).count() < minimumMicroSecViz)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        // read values from iHumanState
        basePositionInterface = iHumanState->getBasePosition();
        baseOrientationInterface = iHumanState->getBaseOrientation();
        jointPositionsInterface = iHumanState->getJointPositions();
        jointNames = iHumanState->getJointNames();

        baseOrientationQuaternion.setVal(0, baseOrientationInterface.at(0));
        baseOrientationQuaternion.setVal(1, baseOrientationInterface.at(1));
        baseOrientationQuaternion.setVal(2, baseOrientationInterface.at(2));
        baseOrientationQuaternion.setVal(3, baseOrientationInterface.at(3));

        basePosition.setVal(0, basePositionInterface.at(0));
        basePosition.setVal(1, basePositionInterface.at(1));
        basePosition.setVal(2, basePositionInterface.at(2));

        wHb.setRotation(iDynTree::Rotation::RotationFromQuaternion(baseOrientationQuaternion));
        wHb.setPosition(basePosition);

        for (size_t iHumanInterfaceIdx = 0; iHumanInterfaceIdx < jointNames.size(); iHumanInterfaceIdx++)
        {
            std::string jointName = jointNames.at(iHumanInterfaceIdx);
            double jointVal = jointPositionsInterface.at(iHumanInterfaceIdx);

            iDynTree::JointIndex jointIndex = viz.modelViz("human").model().getJointIndex(jointName);
            if (jointIndex != iDynTree::JOINT_INVALID_INDEX)
            {
                joints.setVal(jointIndex, jointVal);
            }
        }

        viz.modelViz("human").setPositions(wHb, joints);

        size_t vectorsIterator = 0;
        if (visualizeWrenches)
        {
            for (size_t vectorIndex = 0; vectorIndex < wrenchSourceLinks.size(); vectorIndex++)
            {
                linkTransform = viz.modelViz("human").getWorldLinkTransform(wrenchSourceLinkIndices.at(vectorIndex));
                for (size_t i = 0; i < 3; i++)
                {
                    force.setVal(i, forceScalingFactor * iHumanWrench->getWrenches()[6 * vectorIndex + i]);
                }
                force = linkTransform.getRotation() * force;
                viz.vectors().updateVector(vectorIndex, linkTransform.getPosition(), force);
                vectorsIterator++;
            }
        }

        size_t framesIterator = 0;
        if (visualizeFrames)
        {
            for (size_t vectorIndex = 0; vectorIndex < visualizedLinksFrame.size(); vectorIndex++)
            {
                linkTransform = viz.modelViz("human").getWorldLinkTransform(visualizedLinksFrame.at(vectorIndex));
                viz.frames().updateFrame(vectorIndex, linkTransform);
                framesIterator++;
            }
        }

        if (visualizeTargets)
        {
            for (auto targetEntity : targets)
            {
                switch (targetEntity.second.get()->targetType)
                {
                case hde::KinematicTargetType::pose: case hde::KinematicTargetType::poseAndVelocity: 
                    linkTransform = iDynTree::Transform(targetEntity.second.get()->getCalibratedRotation(), iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                    break;
                case hde::KinematicTargetType::position: case hde::KinematicTargetType::positionAndVelocity:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    linkTransform.setPosition(iDynTree::Position(targetEntity.second.get()->getCalibratedPosition()));
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                    break;
                case hde::KinematicTargetType::orientation: case hde::KinematicTargetType::orientationAndVelocity:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    linkTransform.setRotation(targetEntity.second.get()->getCalibratedRotation());
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                    break;
                case hde::KinematicTargetType::gravity:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    iDynTree::toEigen(gravityVector) = targetsFrameScalingFactor * (iDynTree::toEigen(linkTransform.getRotation()) * iDynTree::toEigen(targetEntity.second.get()->getCalibratedRotation()).row(2).transpose());
                    viz.vectors().updateVector(vectorsIterator, linkTransform.getPosition(), gravityVector);
                    vectorsIterator++;
                    break;
                case hde::KinematicTargetType::floorContact:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                    iDynTree::toEigen(contactForceVector) = targetsForceScalingFactor * iDynTree::toEigen(targetEntity.second.get()->contactForce);
                    viz.vectors().updateVector(vectorsIterator, linkTransform.getPosition(), contactForceVector);
                    contactActive = targetEntity.second.get()->contactActive;
                    if (contactActive)
                    {
                        viz.vectors().setVectorColor(vectorsIterator, iDynTree::ColorViz(1.0, 0.0, 0.0, 1.0));
                    }
                    else
                    {
                        viz.vectors().setVectorColor(vectorsIterator, iDynTree::ColorViz(0.0, 1.0, 0.0, 1.0));
                    }
                    vectorsIterator++;
                    break;
                default:
                    linkTransform = viz.modelViz("human").getWorldLinkTransform(targetEntity.second.get()->modelLinkName);
                    viz.frames().updateFrame(framesIterator, linkTransform);
                    framesIterator++;
                }
            }
        }
        if (visualizeEfforts)
        {
            jointTorques = iHumanDynamics->getJointTorques();
            float minR = 0.0, minG = 1.0, minB = 0.0;
            float maxR = 1.0, maxG = 0.0, maxB = 0.0;

            for (size_t i = 0; i < modelEffortData.size(); ++i) {
                auto& jointEffortData = modelEffortData[i];

                double effortTmp = 0;

                // Compute the module of the Joint Torque
                for (const auto& modelFakeJointIdx : jointEffortData.fakeJointsIndices) {
                    effortTmp += pow(jointTorques.at(modelFakeJointIdx), 2);
                }
                effortTmp = sqrt(effortTmp);

                jointEffortData.effort = effortTmp;

                double effortWeight = jointEffortData.effort / jointEffortData.effortMax;
                if (effortWeight > 1.0) effortWeight = 1.0;

                double r = minR + (maxR - minR) * effortWeight;
                double g = minG + (maxG - minG) * effortWeight;
                double b = minB + (maxB - minB) * effortWeight;

                iDynTree::ColorViz color(r, g, b, 1.0);
                viz.shapes().setShapeColor(i, color);
            }
        }

        // follow the desired link with the camera
        if ( !useFixedCamera )
        {
            cameraDeltaPosition = viz.camera().getPosition() - basePositionOld;
            viz.camera().setPosition(basePosition + cameraDeltaPosition);
            viz.camera().setTarget(basePosition);

            basePositionOld = basePosition;
        }
        
        // Update the visualizer
        viz.draw();
        lastViz = std::chrono::steady_clock::now();
    }

    viz.close();
    humanStateClientDevice.close();
    wearableTargetsClientDevice.close();
    humanWrenchClientDevice.close();

    return 0;
}
