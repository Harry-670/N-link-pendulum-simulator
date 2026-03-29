#pragma once

class Vector {
private:
    // components
    float x, y, z;
    // screen coordinates
    float screenx, screeny;

public:
    // Constructors
    Vector(float x, float y, float z, float screenx = 800.0f, float screeny = 800.0f)
        : x(x), y(y), z(z), screenx(screenx), screeny(screeny) {
    };

    Vector() : Vector(0.0f, 0.0f, 0.0f, 0.0f, 0.0f) {};

    // Unary Operators
    Vector operator+() const;
    Vector operator-() const;

    // Arithmetic Operators
    Vector operator+(const Vector& v) const;
    Vector operator-(const Vector& v) const;
    Vector operator*(const float scalar) const;

    // Friend function for scalar * Vector
    friend Vector operator*(const float scalar, const Vector& v);

    // Assignment Operators (Must return references to work correctly)
    Vector& operator=(const Vector& v);
    Vector& operator+=(const Vector& v);
    Vector& operator-=(const Vector& v);

    // Methods
    void normalize();

    // Getters (Marked const)
    float getx() const { return x; }
    float gety() const { return y; }
    float getz() const { return z; }
    float getScreenX() const { return screenx; }
    float getScreenY() const { return screeny; }

    // Setters
    void setx(float x) { this->x = x; }
    void sety(float y) { this->y = y; }
    void setz(float z) { this->z = z; }
    void setScreen(float screenx, float screeny);
};