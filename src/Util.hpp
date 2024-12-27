#pragma once

#include "FPType.hpp"
#include <thread>

inline thread_local uint32_t RandState = std::hash<std::thread::id>{}(std::this_thread::get_id());

inline uint32_t RandXor() {
    RandState ^= (RandState << 13);
    RandState ^= (RandState >> 17);
    RandState ^= (RandState << 5);
    return RandState;
}

inline FP RandomFP() {
    return (FP) RandXor() / UINT32_MAX;
}

inline FP RandomFP(double a, double b) {
    return a + RandomFP() * (b - a);
}