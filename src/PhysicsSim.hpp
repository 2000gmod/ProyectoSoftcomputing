#pragma once

#include "Drone.hpp"

struct PhysicsSim {
    Drone* SimDrone = nullptr;
    std::array<FP, 2> RequestedThrust = {0};

    PhysicsSim() = default;
    PhysicsSim(Drone& drone) : SimDrone(&drone) { Reset(); }

    void DoSimulationStep(FP deltaT);

    void ManualControlStep(FP left, FP right);
    void NetworkControlStep(const Vec2& target);

    void Reset();
};