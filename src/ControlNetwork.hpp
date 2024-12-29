#pragma once

#include <array>

#include "Config.hpp"

class ControlNetwork {
    public:
        enum class InitMode {
            Zeroes,
            Random,
        };

    private:
        friend class TrainingSim;

        std::array<std::array<FP, InputSize>, Hidden1Size> InToH1Weights;
        std::array<std::array<FP, Hidden1Size>, Hidden2Size> H1ToH2Weights;
        std::array<std::array<FP, Hidden2Size>, OutputSize> H2ToOutWeights;

        std::array<FP, Hidden1Size> H1Biases;
        std::array<FP, Hidden2Size> H2Biases;
        std::array<FP, OutputSize> OutBiases;

    public:
        ControlNetwork(InitMode mode = InitMode::Random);
        ControlNetwork(const ControlNetwork& other);

        ControlNetwork& operator =(const ControlNetwork& other);

        std::array<FP, OutputSize> EvaluateNetwork(const std::array<FP, InputSize>& input);

        static ControlNetwork GenerateChild(FP mRate, const ControlNetwork& a, const ControlNetwork& b);

        FP GetAbsoluteNetworkWeight();
        
    private:
        void InitZeroes();
        void InitRandom();

        template <class F>
        requires requires (F func, double& i) {
            func(i);
        }
        void ApplyForEachValue(const F& func); 
};

template <class F>
requires requires (F func, double& i) {
    func(i);
}
void ControlNetwork::ApplyForEachValue(const F& func) {
    for (auto& iarr : InToH1Weights) {
        for (auto& i : iarr) {
            func(i);
        }
    }

    for (auto& iarr : H1ToH2Weights) {
        for (auto& i : iarr) {
            func(i);
        }
    }

    for (auto& iarr : H2ToOutWeights) {
        for (auto& i : iarr) {
            func(i);
        }
    }

    for (auto& i : H1Biases) {
        func(i);
    }

    for (auto& i : H2Biases) {
        func(i);
    }

    for (auto& i : OutBiases) {
        func(i);
    }
}
        