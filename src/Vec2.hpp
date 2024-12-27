#pragma once

#include "FPType.hpp"

struct Vec2 {
    FP x;
    FP y;

    constexpr Vec2() : x(0.0), y(0.0) { }
    constexpr Vec2(FP scalar) : x(scalar), y(scalar) { }
    constexpr Vec2(FP x, FP y) : x(x), y(y) { }
    constexpr Vec2(const Vec2& b) : x(b.x), y(b.y) { }

    constexpr void operator = (const Vec2& other) {
        x = other.x;
        y = other.y;
    }

    FP Mag() const;
    FP Mag2() const;

    Vec2 Rotated(FP angle);

    Vec2& operator += (const Vec2& other);
};

Vec2 operator + (const Vec2& a, const Vec2& b);
Vec2 operator - (const Vec2& a, const Vec2& b);

Vec2 operator * (const Vec2& a, FP scalar);
Vec2 operator / (const Vec2& a, FP scalar);