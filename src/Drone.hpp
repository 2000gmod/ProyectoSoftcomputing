#pragma once

#include "ControlNetwork.hpp"
#include "Vec2.hpp"


struct Drone {
    ControlNetwork Brain;
    Vec2 Position = {0};
    Vec2 Velocity = {0};
    FP DirectionAngle = 0;
    FP AngularVelocity = 0;

    mutable FP TrainingScore = 1e10;

    Drone() = default;
    Drone(const ControlNetwork& init) : Brain(init) { }
    Drone& operator = (const Drone& other) {
        Brain = other.Brain;
        TrainingScore = other.TrainingScore;
        return *this;
    }
};