#include "Vec2.hpp"

#include <cmath>

FP Vec2::Mag() const {
    return std::sqrt(Mag2());
}

FP Vec2::Mag2() const {
    return x * x + y * y;
}

Vec2 Vec2::Rotated(FP angle) {
    return {
        x * std::cos(angle) - y * std::sin(angle),
        x * std::sin(angle) + y * std::cos(angle),
    };
}

Vec2& Vec2::operator += (const Vec2& other) {
    x += other.x;
    y += other.y;
    return *this;
}

Vec2 operator + (const Vec2& a, const Vec2& b) {
    return {a.x + b.x, a.y + b.y};
}

Vec2 operator - (const Vec2& a, const Vec2& b) {
    return {a.x - b.x, a.y - b.y};
}

Vec2 operator * (const Vec2& a, FP scalar) {
    return {a.x * scalar, a.y * scalar};
}

Vec2 operator / (const Vec2& a, FP scalar) {
    return {a.x / scalar, a.y / scalar};
}