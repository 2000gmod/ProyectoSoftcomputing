#pragma once

#include "FPType.hpp"
#include "Vec2.hpp"

constexpr unsigned int InputSize = 7;
constexpr unsigned int Hidden1Size = 10;
constexpr unsigned int Hidden2Size = 5;
constexpr unsigned int OutputSize = 2;

constexpr FP PhysicsSimDeltaT = 1.0 / 60.0;
constexpr FP PhysicsSimTargetDroneSpeed = 1.5;

constexpr Vec2 Gravity = {0.0, -10.0};
constexpr FP DroneMass = 10.0;
constexpr FP DroneMomentOfInertia = 5.0;
constexpr FP DroneThrust = 35.0;
constexpr FP DroneTorqueMultiplier = 60.0;
constexpr FP DroneThrustChangeSpeed = 8;

constexpr FP DroneWidth = 0.5;
constexpr FP DroneHeight = 0.25;

//constexpr FP MutationRate = 1e-3;

constexpr unsigned int GenerationSize = 1000;
constexpr unsigned int SelectNBest = 50;
constexpr unsigned int SimulationsPerDrone = 10;
constexpr unsigned int SimulationThreads = 4;

constexpr FP TrainingDistancePenaltyWeight = 20.0;
constexpr FP TrainingSpeedPenaltyWeight = 20.0;
constexpr FP TrainingAnglePenaltyWeight = 5.0;
constexpr FP TrainingAngularVelPenaltyWeight = 10.0;
constexpr FP TrainingNetworkWeightPenalty = 0.05;

constexpr FP TrainingMaxCoords = 4.0;

constexpr bool TrainingUseRandomInitConditions = false;

constexpr const char* CheckpointFileName = "checkpoint.gen";


static_assert(InputSize == 7 && OutputSize == 2);
static_assert(SelectNBest <= GenerationSize);
static_assert(GenerationSize % SimulationThreads == 0);