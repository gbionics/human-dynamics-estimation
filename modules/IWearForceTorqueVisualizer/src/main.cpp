// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#include <iDynTree/Visualizer.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>

#include <Wearable/IWear/IWear.h>
#include <Wearable/IWear/Sensors/IForceTorque6DSensor.h>

#include <atomic>
#include <csignal>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace yarp::os;

// ============================================================
// Signal handling
// ============================================================

std::atomic<bool> isClosing{false};

void my_handler(int)
{
    isClosing = true;
}

#ifdef WIN32
#include <windows.h>
BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType) {
        case CTRL_C_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            my_handler(0);
            return TRUE;
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

// ============================================================
// Per-sensor visualization data
// ============================================================

struct FTSensorViewer
{
    wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor> sensor;
    iDynTree::Position offsetPosition; ///< World-frame base of the force arrow
    iDynTree::Rotation rotation;       ///< Sensor-frame → world-frame rotation
    size_t vectorIndex;                ///< Index returned by viz.vectors().addVector()
};

// ============================================================
// main
// ============================================================

int main(int argc, char* argv[])
{
    const std::string logPrefix = "IWearForceTorqueVisualizer";

    handleSigInt();

    // -------------------------
    // Resource finder / config
    // -------------------------
    yarp::os::ResourceFinder& config = yarp::os::ResourceFinder::getResourceFinderSingleton();
    config.setDefaultConfigFile("IWearForceTorqueVisualizerConfig.ini");
    config.configure(argc, argv);

    // -------------------------
    // Visualizer
    // -------------------------
    iDynTree::Visualizer visualizer;

    if (!visualizer.init()) {
        yError() << logPrefix << "Failed to initialize the iDynTree visualizer.";
        return EXIT_FAILURE;
    }
    visualizer.camera().animator()->enableMouseControl();

    // -------------------------
    // IWear remapper driver
    // -------------------------
    yarp::dev::PolyDriver iWearDriver;
    wearable::IWear* iWear{nullptr};

    yarp::os::Value* wearableDataPort;
    if (!config.check("wearable_data_ports", wearableDataPort)) {
        yError() << logPrefix << "Unable to find wearable_data_ports in the config file.";
        return EXIT_FAILURE;
    }

    yarp::os::Property wearOptions;
    wearOptions.put("device", "iwear_remapper");
    wearOptions.put("wearableDataPorts", wearableDataPort);

    if (!iWearDriver.open(wearOptions)) {
        yError() << logPrefix
                 << "Unable to open PolyDriver with options:" << wearOptions.toString();
        return EXIT_FAILURE;
    }

    if (!iWearDriver.view(iWear) || iWear == nullptr) {
        yError() << logPrefix << "Unable to view IWear interface from the remapper.";
        return EXIT_FAILURE;
    }

    // Wait until the interface is ready
    while (iWear->getStatus() == wearable::WearStatus::WaitingForFirstRead) {
        yInfo() << logPrefix << "IWear interface waiting for first data. Waiting...";
        yarp::os::Time::delay(0.1);
    }

    if (iWear->getStatus() != wearable::WearStatus::Ok) {
        yError() << logPrefix << "IWear interface status is not Ok ("
                 << static_cast<int>(iWear->getStatus()) << ").";
        return EXIT_FAILURE;
    }

    // -------------------------
    // Collect FT sensors
    // -------------------------
    auto allFTSensors = iWear->getForceTorque6DSensors();

    if (allFTSensors.empty()) {
        yError() << logPrefix << "No IForceTorque6DSensor found on the IWear interface.";
        return EXIT_FAILURE;
    }

    // Build a name→sensor map for quick lookup
    std::unordered_map<std::string,
                       wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>>
        sensorsByName;
    for (auto& s : allFTSensors) {
        sensorsByName[s->getSensorName()] = s;
    }

    // Optional whitelist filter
    std::vector<wearable::SensorPtr<const wearable::sensor::IForceTorque6DSensor>> selectedSensors;

    if (config.check("sensor_names_filter") && config.find("sensor_names_filter").isList()) {
        yarp::os::Bottle* filterList = config.find("sensor_names_filter").asList();
        for (size_t i = 0; i < filterList->size(); ++i) {
            const std::string name = filterList->get(i).asString();
            auto it = sensorsByName.find(name);
            if (it == sensorsByName.end()) {
                yError() << logPrefix << "Sensor listed in sensor_names_filter not found:" << name;
                return EXIT_FAILURE;
            }
            selectedSensors.push_back(it->second);
        }
        yInfo() << logPrefix << "Using" << selectedSensors.size()
                << "sensor(s) from sensor_names_filter.";
    }
    else {
        selectedSensors = allFTSensors;
        yInfo() << logPrefix << "No sensor_names_filter found. Using all"
                << selectedSensors.size() << "IForceTorque6DSensor(s).";
    }

    // -------------------------
    // Force scaling factor
    // -------------------------
    const double forceScalingFactor =
        config.check("force_scaling_factor", yarp::os::Value(0.001)).asFloat64();
    yInfo() << logPrefix << "force_scaling_factor:" << forceScalingFactor;

    // -------------------------
    // Parse per-sensor config overrides
    // Format: ((sensorName (px py pz) (R00..R22)) ...)
    // -------------------------
    // name → (offset, rotation)
    std::unordered_map<std::string,
                       std::pair<iDynTree::Position, iDynTree::Rotation>>
        sensorConfig;

    if (config.check("sensors_config") && config.find("sensors_config").isList()) {
        yarp::os::Bottle* sensorsConfigBottle = config.find("sensors_config").asList();
        for (size_t i = 0; i < sensorsConfigBottle->size(); ++i) {
            yarp::os::Bottle* entry = sensorsConfigBottle->get(i).asList();
            if (!entry || entry->size() < 1) {
                yError() << logPrefix << "sensors_config entry" << i
                         << "is malformed (expected at least a sensor name).";
                return EXIT_FAILURE;
            }

            const std::string sensorName = entry->get(0).asString();

            // Default: zero offset, identity rotation
            iDynTree::Position offset = iDynTree::Position::Zero();
            iDynTree::Rotation rotation = iDynTree::Rotation::Identity();

            // Parse optional offset (px py pz)
            if (entry->size() >= 2 && entry->get(1).isList()) {
                yarp::os::Bottle* posList = entry->get(1).asList();
                if (posList->size() == 3) {
                    offset = iDynTree::Position(posList->get(0).asFloat64(),
                                                posList->get(1).asFloat64(),
                                                posList->get(2).asFloat64());
                }
                else {
                    yWarning() << logPrefix << "sensors_config entry for" << sensorName
                               << ": position list has" << posList->size()
                               << "elements (expected 3). Using zero offset.";
                }
            }

            // Parse optional rotation matrix (9 elements, row-major)
            if (entry->size() >= 3 && entry->get(2).isList()) {
                yarp::os::Bottle* rotList = entry->get(2).asList();
                if (rotList->size() == 9) {
                    rotation = iDynTree::Rotation(rotList->get(0).asFloat64(),
                                                  rotList->get(1).asFloat64(),
                                                  rotList->get(2).asFloat64(),
                                                  rotList->get(3).asFloat64(),
                                                  rotList->get(4).asFloat64(),
                                                  rotList->get(5).asFloat64(),
                                                  rotList->get(6).asFloat64(),
                                                  rotList->get(7).asFloat64(),
                                                  rotList->get(8).asFloat64());
                }
                else {
                    yWarning() << logPrefix << "sensors_config entry for" << sensorName
                               << ": rotation list has" << rotList->size()
                               << "elements (expected 9). Using identity rotation.";
                }
            }

            sensorConfig[sensorName] = {offset, rotation};
        }
    }

    // -------------------------
    // Build FTSensorViewer list and register vectors in the visualizer
    // -------------------------
    std::vector<FTSensorViewer> sensorViewers;
    sensorViewers.reserve(selectedSensors.size());

    // Dummy zero vector for initial addVector call
    iDynTree::Vector3 zeroVec;
    zeroVec.zero();

    for (auto& sensor : selectedSensors) {
        FTSensorViewer viewer;
        viewer.sensor = sensor;

        // Look up per-sensor config; fall back to identity/zero
        auto cfgIt = sensorConfig.find(sensor->getSensorName());
        if (cfgIt != sensorConfig.end()) {
            viewer.offsetPosition = cfgIt->second.first;
            viewer.rotation       = cfgIt->second.second;
        }
        else {
            viewer.offsetPosition = iDynTree::Position::Zero();
            viewer.rotation       = iDynTree::Rotation::Identity();
        }

        viewer.vectorIndex = visualizer.vectors().addVector(viewer.offsetPosition, zeroVec);

        yInfo() << logPrefix << "Registered sensor:" << sensor->getSensorName()
                << "  offset:" << viewer.offsetPosition.toString()
                << "  vectorIndex:" << viewer.vectorIndex;

        sensorViewers.push_back(std::move(viewer));
    }

    yInfo() << logPrefix << "=================";
    yInfo() << logPrefix << "===  Running  ===";
    yInfo() << logPrefix << "=================";

    // -------------------------
    // Visualization loop
    // -------------------------
    iDynTree::Vector3 forceWorld;
    wearable::Vector3 rawForce;

    while (visualizer.run() && !isClosing) {
        for (auto& viewer : sensorViewers) {
            if (viewer.sensor->getSensorStatus() != wearable::WearStatus::Ok) {
                yWarning() << logPrefix
                           << "Sensor status not Ok:" << viewer.sensor->getSensorName();
                continue;
            }

            viewer.sensor->getForceTorque3DForce(rawForce);

            // Build iDynTree vector from raw force and apply sensor→world rotation
            iDynTree::Vector3 forceSensor;
            forceSensor.setVal(0, forceScalingFactor * rawForce[0]);
            forceSensor.setVal(1, forceScalingFactor * rawForce[1]);
            forceSensor.setVal(2, forceScalingFactor * rawForce[2]);

            // Apply sensor->world rotation: forceWorld = R * forceSensor
            forceWorld.setVal(0,
                viewer.rotation(0, 0) * forceSensor[0] +
                viewer.rotation(0, 1) * forceSensor[1] +
                viewer.rotation(0, 2) * forceSensor[2]);
            forceWorld.setVal(1,
                viewer.rotation(1, 0) * forceSensor[0] +
                viewer.rotation(1, 1) * forceSensor[1] +
                viewer.rotation(1, 2) * forceSensor[2]);
            forceWorld.setVal(2,
                viewer.rotation(2, 0) * forceSensor[0] +
                viewer.rotation(2, 1) * forceSensor[1] +
                viewer.rotation(2, 2) * forceSensor[2]);

            visualizer.vectors().updateVector(viewer.vectorIndex,
                                              viewer.offsetPosition,
                                              forceWorld);
        }

        visualizer.draw();
    }

    iWearDriver.close();

    return EXIT_SUCCESS;
}
