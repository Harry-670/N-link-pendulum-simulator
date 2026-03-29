#include "Vector.h"
#include <cmath>

// --- Unary Operators ---
Vector Vector::operator+() const {
    return Vector(x, y, z, screenx, screeny);
}

Vector Vector::operator-() const {
    return Vector(-x, -y, -z, screenx, screeny);
}

// --- Arithmetic Operators ---
Vector Vector::operator+(const Vector& v) const {
    return Vector(x + v.x, y + v.y, z + v.z, screenx, screeny);
}

Vector Vector::operator-(const Vector& v) const {
    return Vector(x - v.x, y - v.y, z - v.z, screenx, screeny);
}

Vector Vector::operator*(const float scalar) const {
    return Vector(x * scalar, y * scalar, z * scalar, screenx, screeny);
}

Vector operator*(const float scalar, const Vector& v) {
    return v * scalar;
}

// --- Assignment Operators (Fixed) ---
// Assignment should return a reference to *this and NOT be const
Vector& Vector::operator=(const Vector& v) {
    if (this != &v) {
        x = v.x;
        y = v.y;
        z = v.z;
        screenx = v.screenx;
        screeny = v.screeny;
    }
    return *this;
}

Vector& Vector::operator+=(const Vector& v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vector& Vector::operator-=(const Vector& v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

// --- Methods ---
void Vector::normalize() {
    float mag = std::sqrt(x * x + y * y + z * z);
    if (mag > 0.0f) { // Prevent division by zero
        x /= mag;
        y /= mag;
        z /= mag;
    }
}


void Vector::setScreen(float screenx, float screeny) {
    this->screenx = screenx;
    this->screeny = screeny;
}