#include "ControlNetwork.hpp"
#include "Config.hpp"

#include <cmath>
#include <cstring>
#include <random>

static FP ReLU(FP x) {
    if (x < 0.0) return 0.1 * x;
    return x;
}

static FP Sigmoid(FP x) {
    return 1.0 / (1.0 + std::exp(-x));
}

static FP Act3(FP x) {
    return std::cbrt(x);
}

static const auto ActivationFunc = ReLU;

ControlNetwork::ControlNetwork(ControlNetwork::InitMode mode) {
    switch(mode) {
        case ControlNetwork::InitMode::Zeroes:
            InitZeroes();
            break;
        case ControlNetwork::InitMode::Random:
            InitRandom();
            break;
    }
}

ControlNetwork::ControlNetwork(const ControlNetwork& other) {
    InToH1Weights = other.InToH1Weights;
    H1ToH2Weights = other.H1ToH2Weights;
    H2ToOutWeights = other.H2ToOutWeights;

    H1Biases = other.H1Biases;
    H2Biases = other.H2Biases;
    OutBiases = other.OutBiases;

    /*const FP* begin = &other.InToH1Weights[0][0];
    const FP* afterLast = (&other.OutBiases[other.OutBiases.size() - 1]) + 1;
    std::memcpy(&InToH1Weights[0][0], begin, sizeof(FP) * (afterLast - begin));*/
}

void ControlNetwork::InitZeroes() {
    InToH1Weights.fill({0});
    H1ToH2Weights.fill({0});
    H2ToOutWeights.fill({0});

    H1Biases.fill(0);
    H2Biases.fill(0);
    OutBiases.fill(0);
}

void ControlNetwork::InitRandom() {
    std::random_device rd {};
    std::mt19937 gen {rd()};

    std::normal_distribution<FP> dist {0.0, 1.0};
    
    ApplyForEachValue([&dist, &gen] (double& i) {
        i = dist(gen);
    });
    
}

std::array<FP, OutputSize> ControlNetwork::EvaluateNetwork(const std::array<FP, InputSize>& input) {
    std::array<FP, Hidden1Size> h1activations;

    for (int i = 0; i < Hidden1Size; i++) {
        FP sum = 0;
        for (int j = 0; j < InputSize; j++) {
            sum += input[j] * InToH1Weights[i][j] + H1Biases[i];
        }
        h1activations[i] = ActivationFunc(sum);
    }

    std::array<FP, Hidden2Size> h2Activations;

    for (int i = 0; i < Hidden2Size; i++) {
        FP sum = 0;
        for (int j = 0; j < Hidden1Size; j++) {
            sum += h1activations[j] * H1ToH2Weights[i][j] + H2Biases[i];
        }
        h2Activations[i] = ActivationFunc(sum);
    }

    std::array<FP, OutputSize> outActivations;

    for (int i = 0; i < OutputSize; i++) {
        FP sum = 0;
        for (int j = 0; j < Hidden2Size; j++) {
            sum += h2Activations[j] * H2ToOutWeights[i][j] + OutBiases[i];
        }
        outActivations[i] = ActivationFunc(sum);
    }

    return outActivations;
}

ControlNetwork ControlNetwork::GenerateChild(FP mRate, const ControlNetwork& a, const ControlNetwork& b) {
    static thread_local std::random_device rd {};
    static thread_local std::mt19937 gen {rd()};

    std::normal_distribution<FP> dist {0.0, mRate};
    //static thread_local std::bernoulli_distribution decide;

    ControlNetwork out(InitMode::Zeroes);

    for (int i = 0; i < Hidden1Size; i++) {
        for (int j = 0; j < InputSize; j++) {
            auto value = (a.InToH1Weights[i][j] + b.InToH1Weights[i][j]) / 2;
            //value += dist(gen);
            out.InToH1Weights[i][j] = value;
        }
    }

    for (int i = 0; i < Hidden2Size; i++) {
        for (int j = 0; j < Hidden1Size; j++) {
            auto value = (a.H1ToH2Weights[i][j] + b.H1ToH2Weights[i][j]) / 2;
            //value += dist(gen);
            out.H1ToH2Weights[i][j] = value;
        }
    }

    for (int i = 0; i < OutputSize; i++) {
        for (int j = 0; j < Hidden2Size; j++) {
            auto value = (a.H2ToOutWeights[i][j] + b.H2ToOutWeights[i][j]) / 2;
            //value += dist(gen);
            out.H2ToOutWeights[i][j] = value;
        }
    }

    for (int i = 0; i < Hidden1Size; i++) {
        auto value = (a.H1Biases[i] + b.H1Biases[i]) / 2;
        //value += dist(gen);
        out.H1Biases[i] = value;
    }

    for (int i = 0; i < Hidden2Size; i++) {
        auto value = (a.H2Biases[i] + b.H2Biases[i]) / 2;
        //value += dist(gen);
        out.H2Biases[i] = value;
    }

    for (int i = 0; i < OutputSize; i++) {
        auto value = (a.OutBiases[i] + b.OutBiases[i]) / 2;
        //value += dist(gen);
        out.OutBiases[i] = value;
    }

    FP* begin = &out.InToH1Weights[0][0];
    FP* afterLast = (&out.OutBiases[out.OutBiases.size() - 1]) + 1;
    
    int randomGene = rand() % (afterLast - begin);

    begin[randomGene] += dist(gen);

    return out;
}

FP ControlNetwork::GetAbsoluteNetworkWeight() {
    FP total = 0.0;

    ApplyForEachValue([&total] (FP w) {
        total += std::abs(w);
    });
    
    return total;
}