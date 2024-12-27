#pragma once

#include <vector>
#include "Drone.hpp"

struct TrainingSim {
    std::vector<Drone> Drones;
    int GenerationsDone = 0;

    TrainingSim();

    FP DoDronePerformanceSimulation(Drone& drone);
    FP TrainGeneration();

    void SaveToFile() const;
    void LoadFromFile();
};