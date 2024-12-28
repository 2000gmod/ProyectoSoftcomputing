#include "PhysicsSim.hpp"
#include "Config.hpp"
#include "Vec2.hpp"

#include <cmath>
#include <numbers>
#include <algorithm>

void PhysicsSim::DoSimulationStep(FP deltaT) {
    const auto& [thrustL, thrustR] = RequestedThrust;

    Vec2 totalForces;

    totalForces += Gravity;

    auto thrustForce = Vec2(0.0, thrustL + thrustR).Rotated(SimDrone->DirectionAngle);

    thrustForce = thrustForce * DroneThrust;
    
    totalForces += thrustForce;

    auto acceleration = totalForces / DroneMass;

    SimDrone->Velocity += acceleration * deltaT;
    SimDrone->Position += SimDrone->Velocity * deltaT;

    FP torqueImbalance = (thrustR - thrustL) * DroneTorqueMultiplier;

    FP angularAcceleration = torqueImbalance / DroneMomentOfInertia;

    SimDrone->AngularVelocity += angularAcceleration * deltaT;
    SimDrone->DirectionAngle += SimDrone->AngularVelocity * deltaT;
    SimDrone->DirectionAngle = std::fmod(SimDrone->DirectionAngle, 2.0 * std::numbers::pi);
}

void PhysicsSim::ManualControlStep(FP left, FP right, FP deltaT) {
    //RequestedThrust[0] = std::clamp(left, 0.0, 1.0);
    //RequestedThrust[1] = std::clamp(right, 0.0, 1.0);

    left = std::clamp(left, 0.0, 1.0);
    right = std::clamp(right, 0.0, 1.0);

    FP thrustChange = DroneThrustChangeSpeed * deltaT;

    if (left > RequestedThrust[0] + thrustChange) {
        RequestedThrust[0] += thrustChange;
    }
    else if (left < RequestedThrust[0] - thrustChange) {
        RequestedThrust[0] -= thrustChange;
    }
    else {
        RequestedThrust[0] = left;
    }

    if (right > RequestedThrust[1] + thrustChange) {
        RequestedThrust[1] += thrustChange;
    }
    else if (right < RequestedThrust[1] - thrustChange) {
        RequestedThrust[1] -= thrustChange;
    }
    else {
        RequestedThrust[1] = right;
    }
}

void PhysicsSim::NetworkControlStep(const Vec2& target, FP deltaT) {
    auto difX = SimDrone->Position.x - target.x;
    auto difY = SimDrone->Position.y - target.y;
    auto velX = SimDrone->Velocity.x;
    auto velY = SimDrone->Velocity.y;
    auto angVel = SimDrone->AngularVelocity;
    auto sinAng = std::sin(SimDrone->DirectionAngle);
    auto cosAng = std::cos(SimDrone->DirectionAngle);

    auto thrusters = SimDrone->Brain.EvaluateNetwork({difX, difY, velX, velY, angVel, sinAng, cosAng});
    ManualControlStep(thrusters[0], thrusters[1], deltaT);
}

void PhysicsSim::Reset() {
    SimDrone->Position = 0;
    SimDrone->Velocity = 0;
    SimDrone->AngularVelocity = 0;
    SimDrone->DirectionAngle = 0;
}