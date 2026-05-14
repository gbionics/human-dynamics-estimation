// SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
// SPDX-License-Identifier: BSD-3-Clause

#ifndef HDE_INTERFACES_IWEARABLETARGETS
#define HDE_INTERFACES_IWEARABLETARGETS

#include <array>
#include <string>
#include <vector>
#include <mutex>

#include <iDynTree/SpatialVector.h>
#include <iDynTree/Transform.h>
#include <iDynTree/Twist.h>
#include <Wearable/IWear/IWear.h>

namespace hde {

    enum KinematicTargetType
    {
        none,
        pose,
        poseAndVelocity,
        position,
        positionAndVelocity,
        orientation,
        orientationAndVelocity,
        gravity,
        floorContact
    };

    using TargetName = std::string;
    using ModelLinkName = std::string;

    class WearableSensorTarget
    {
        public:
        wearable::WearableName wearableName;
        ModelLinkName modelLinkName;
        hde::KinematicTargetType targetType;

        iDynTree::Vector3 position;
        iDynTree::Rotation rotation;
        iDynTree::Vector3 linearVelocity;
        iDynTree::Vector3 angularVelocity;

        iDynTree::Transform calibrationWorldToMeasurementWorld;
        iDynTree::Transform calibrationMeasurementToLink;
        iDynTree::Transform calibrationMeasurementToLinkInitial;
        iDynTree::Vector3 positionScaleFactor;

        // buffer variables
        iDynTree::Vector3 positionInWorld;
        iDynTree::Vector3 positionInMeasurementWorld;
        iDynTree::Vector3 positionScaled;
        iDynTree::Vector3 linearVelocityInWorld;
        iDynTree::Vector3 linearVelocityInMeasurementWorld;
        iDynTree::Vector3 linearVelocityScaled;
        iDynTree::Twist twistMeasured;
        iDynTree::Transform mixedTransform;

        // target-specific configurations
        // floor contact
        bool contactActive;
        double contactThreshold = 0; // [N]
        iDynTree::Vector3 contactForce; // [N]

        mutable std::mutex mutex;

        WearableSensorTarget(wearable::WearableName wearableName_,
                            ModelLinkName modelLinkName_,
                            hde::KinematicTargetType targetType_)
                            : wearableName(wearableName_)
                            , modelLinkName(modelLinkName_)
                            , targetType(targetType_)
                            , rotation(iDynTree::Rotation::Identity())
                            , mixedTransform(iDynTree::Transform::Identity())
                            , contactActive(true)
        {
            position.zero();
            linearVelocity.zero();
            angularVelocity.zero();
            positionScaleFactor(0) = 1;
            positionScaleFactor(1) = 1;
            positionScaleFactor(2) = 1;

            clearCalibrationMatrices();
        };

        void clearCalibrationMatrices()
        {
            std::lock_guard<std::mutex> lock(mutex);
            calibrationWorldToMeasurementWorld = iDynTree::Transform::Identity();
            calibrationMeasurementToLink = calibrationMeasurementToLinkInitial;
        };

        void clearWorldCalibrationMatrix()
        {
            std::lock_guard<std::mutex> lock(mutex);
            calibrationWorldToMeasurementWorld = iDynTree::Transform::Identity();
        }

        void clearSecondaryCalibrationMatrix()
        {
            std::lock_guard<std::mutex> lock(mutex);
            calibrationMeasurementToLink = iDynTree::Transform::Identity();
        }

        iDynTree::Vector3 getCalibratedPosition()
        {
            std::lock_guard<std::mutex> lock(mutex);
            positionInMeasurementWorld = iDynTree::Position(calibrationMeasurementToLink.getPosition()).changeCoordinateFrame(rotation);
            positionInWorld = (iDynTree::Position(positionInMeasurementWorld) + iDynTree::Position(position)).changeCoordinateFrame(calibrationWorldToMeasurementWorld.getRotation()) + calibrationWorldToMeasurementWorld.getPosition();
            // scale position
            positionScaled.setVal(0, positionInWorld.getVal(0) * positionScaleFactor(0));
            positionScaled.setVal(1, positionInWorld.getVal(1) * positionScaleFactor(1));
            positionScaled.setVal(2, positionInWorld.getVal(2) * positionScaleFactor(2));

            return positionScaled;
        };

        iDynTree::Rotation getCalibratedRotation()
        {
            std::lock_guard<std::mutex> lock(mutex);
            return calibrationWorldToMeasurementWorld.getRotation() * rotation * calibrationMeasurementToLink.getRotation();
        };

        iDynTree::Vector3 getCalibratedLinearVelocity()
        {
            std::lock_guard<std::mutex> lock(mutex);
            mixedTransform.setPosition(calibrationMeasurementToLink.inverse().getPosition());
            twistMeasured.setLinearVec3(linearVelocity);
            twistMeasured.setAngularVec3(angularVelocity);
            linearVelocityInMeasurementWorld = (mixedTransform*twistMeasured).getLinearVec3();
            linearVelocityInWorld = iDynTree::LinearMotionVector3(linearVelocityInMeasurementWorld).changeCoordFrame(calibrationWorldToMeasurementWorld.getRotation());
            // scale linear velocity
            linearVelocityScaled.setVal(0, linearVelocityInWorld.getVal(0) * positionScaleFactor(0));
            linearVelocityScaled.setVal(1, linearVelocityInWorld.getVal(1) * positionScaleFactor(1));
            linearVelocityScaled.setVal(2, linearVelocityInWorld.getVal(2) * positionScaleFactor(2));
            return linearVelocityScaled;
        };

        iDynTree::Vector3 getCalibratedAngularVelocity()
        {
            std::lock_guard<std::mutex> lock(mutex);
            return iDynTree::AngularMotionVector3(angularVelocity).changeCoordFrame(calibrationWorldToMeasurementWorld.getRotation());
        };
    };

    namespace interfaces {
        class IWearableTargets;
    } // namespace interfaces
} // namespace hde

class hde::interfaces::IWearableTargets
{
public:
    virtual ~IWearableTargets() = default;

    virtual std::vector<TargetName> getAllTargetsName() const = 0;

    virtual std::shared_ptr<hde::WearableSensorTarget> getTarget(const TargetName name) const = 0;
};

#endif // HDE_INTERFACES_IWEARABLETARGETS
