#include "TrainingSim.hpp"
#include "Config.hpp"
#include "ControlNetwork.hpp"
#include "Drone.hpp"
#include "PhysicsSim.hpp"
#include "Util.hpp"

#include <ThreadPool.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <fstream>

TrainingSim::TrainingSim() {
    Drones.reserve(GenerationSize);

    for (int i = 0; i < (int) GenerationSize; i++) {
        Drones.emplace_back();
    }
}

FP TrainingSim::DoDronePerformanceSimulation(Drone& drone) {
    PhysicsSim sim(drone);

    FP penaltyScore = 0.0;
    for (int i = 0; i < (int) SimulationsPerDrone; i++) {
        sim.Reset();
        Vec2 target {RandomFP(-TrainingMaxCoords, TrainingMaxCoords), RandomFP(-TrainingMaxCoords, TrainingMaxCoords)};
        //Vec2 target;

        FP timeLimit = target.Mag() / PhysicsSimTargetDroneSpeed + 1.5;

        if constexpr (TrainingUseRandomInitConditions) {
            drone.AngularVelocity = RandomFP(-1, 1);
            drone.Velocity.x = RandomFP(-1, 1);
            drone.Velocity.y = RandomFP(-1, 1);
            drone.DirectionAngle = RandomFP(-1, 1);
        }

        for (FP t = 0.0; t < timeLimit; t += PhysicsSimDeltaT) {
            sim.NetworkControlStep(target, PhysicsSimDeltaT);
            sim.DoSimulationStep(PhysicsSimDeltaT);
        }

        FP simPenaltyScore = (sim.SimDrone->Position - target).Mag2() * TrainingDistancePenaltyWeight;
        simPenaltyScore += sim.SimDrone->Velocity.Mag() * TrainingSpeedPenaltyWeight;
        simPenaltyScore += std::abs(std::min(sim.SimDrone->DirectionAngle, 2 * std::numbers::pi - sim.SimDrone->DirectionAngle) * TrainingAnglePenaltyWeight);
        simPenaltyScore += std::abs(sim.SimDrone->AngularVelocity) * TrainingAngularVelPenaltyWeight;

        penaltyScore += simPenaltyScore / SimulationsPerDrone;
    }

    //penaltyScore += drone.Brain.GetAbsoluteNetworkWeight() * TrainingNetworkWeightPenalty;

    drone.TrainingScore = penaltyScore;
    return penaltyScore;
}

static ll::ThreadPool pool {SimulationThreads};

FP TrainingSim::TrainGeneration() {
    std::atomic<FP> avgPenalty = 0;

    auto dronesPerThread = GenerationSize / SimulationThreads;

    static auto worker = [this, &avgPenalty] (int numDrones, int startIndex) {
        for (int i = startIndex; i < startIndex + numDrones; i++) {
            avgPenalty += DoDronePerformanceSimulation(Drones[i]);
        }
    };

    for (int i = 0; i < (int) SimulationThreads; i++) {
        pool.Submit(worker, dronesPerThread, i * dronesPerThread);
    }
    pool.WaitUntilEmpty();

    avgPenalty = avgPenalty / GenerationSize;

    std::sort(Drones.begin(), Drones.end(), [] (Drone& a, Drone& b) {
        return a.TrainingScore < b.TrainingScore;
    });

    Drones.erase(Drones.begin() + SelectNBest, Drones.end());

    /*for (int i = 0; i < SelectNBest; i++) {
        auto numCrosses = GenerationSize / SelectNBest;
        
        for (int j = 0; j < numCrosses; j++) {
            auto secondIndex = 0;
            if (i + j > SelectNBest) secondIndex = i - j;
            else secondIndex = i + j;
            Drones.emplace_back(ControlNetwork::GenerateChild(Drones[i].Brain, Drones[secondIndex].Brain));
        }
    }*/

    static thread_local std::random_device rd {};
    static thread_local std::mt19937 gen {rd()};

    std::geometric_distribution<int> geom(Drones[0].TrainingScore / Drones[1].TrainingScore);

    auto bestScore = Drones[0].TrainingScore;

    FP mutRate = std::min(std::pow(bestScore, 0.33), 3.0);
    //FP mutRate = std::min(std::pow(10.0, bestScore - 2), 1e-1);


    while (Drones.size() < GenerationSize) {
        auto index1 = std::min(geom(gen), (int) SelectNBest);
        auto index2 = std::min(geom(gen), (int) SelectNBest);


        Drones.emplace_back(ControlNetwork::GenerateChild(mutRate, Drones[index1].Brain, Drones[index2].Brain));
    }

    GenerationsDone++;
    return avgPenalty;
}

void TrainingSim::SaveToFile() const {
    std::ofstream file {CheckpointFileName};

    file << GenerationsDone << "\n";
    file << Hidden1Size << "\n";
    file << Hidden2Size << "\n";

    for (auto& drone : Drones) {
        auto& brain = drone.Brain;

        for (auto& arr : brain.InToH1Weights) {
            for (auto& i : arr) file << i << "\n";
        }

        for (auto& arr : brain.H1ToH2Weights) {
            for (auto& i : arr) file << i << "\n";
        }

        for (auto& arr : brain.H2ToOutWeights) {
            for (auto& i : arr) file << i << "\n";
        }

        for (auto& i : brain.H1Biases) file << i << "\n";
        for (auto& i : brain.H2Biases) file << i << "\n";
        for (auto& i : brain.OutBiases) file << i << "\n";
    }
    std::cout << "Saved to checkpoint file." << std::endl;
}

void TrainingSim::LoadFromFile() {
    std::ifstream file {CheckpointFileName};

    if (!file.is_open()) {
        std::cerr << "Could not open checkpoint file, skipping." << std::endl;
        return;
    }

    int gens;
    file >> gens;

    int h1, h2;

    file >> h1;
    file >> h2;

    if (h1 != Hidden1Size || h2 != Hidden2Size) {
        std::cerr << "Incorrect checkpoint architecture, aborting read." << std::endl;
        return;
    }

    GenerationsDone = gens;

    for (auto& drone : Drones) {
        auto& brain = drone.Brain;

        for (auto& arr : brain.InToH1Weights) {
            for (auto& i : arr) file >> i;
        }

        for (auto& arr : brain.H1ToH2Weights) {
            for (auto& i : arr) file >> i;
        }

        for (auto& arr : brain.H2ToOutWeights) {
            for (auto& i : arr) file >> i;
        }

        for (auto& i : brain.H1Biases) file >> i;
        for (auto& i : brain.H2Biases) file >> i;
        for (auto& i : brain.OutBiases) file >> i;
    }

    std::cout << "Loaded checkpoint file." << std::endl;
}