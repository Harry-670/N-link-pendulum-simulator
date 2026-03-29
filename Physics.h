#ifndef PHYSICS_H
#define PHYSICS_H

#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define grav 1.0f

class Particle {
private:

public:

    glm::vec3 pos, posPrev, vel, acc, pivot;
    float angle, radius, length,angVel, angAcc;
    float red, green, blue;
    unsigned int nodes;

    // Constructors
    Particle(glm::vec3 pos, glm::vec3 posPrev, glm::vec3 vel, glm::vec3 acc, glm::vec3 pivot,
        float angle, float radius, float length, float angVel, float angAcc,
        float red, float green, float blue, unsigned int nodes)
        : pos(pos), posPrev(posPrev), vel(vel), acc(acc), pivot(pivot),
        angle(angle), radius(radius), length(length), angVel(angVel), angAcc(angAcc),
        red(red), green(green), blue(blue), nodes(nodes) {
    };

    Particle() : Particle(glm::vec3(), glm::vec3(), glm::vec3(), glm::vec3(), glm::vec3(),
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0) {
    };

    // Physics Logic
    void newStep(float timeStep);
    float* initCircle();
    glm::vec3 const polarToCartVert();

    // --- Setters ---

    void setNodes(unsigned int nodes) { this->nodes = nodes; }
    void setAngle(float angle) { this->angle = angle; }
    void setRadius(float radius) { this->radius = radius; }
    void setAngVel(float angVel){this->angVel = angVel;}
    void setAngAcc(float angAcc) { this->angAcc = angAcc; }
    void setLength(float length) { this->length = length; }

    // Color Setters
    void setRed(float r) { red = r; }
    void setGreen(float g) { green = g; }
    void setBlue(float b) { blue = b; }

    // --- Getters ---

    float getAngle() const { return angle; }
    float getR() const { return radius; }
    float getAngVel() const { return angVel; }
    float getAngAcc() const { return angAcc; }
    unsigned int getNodes() const { return nodes; }

    // Color Getters
    float getRed() const { return red; }
    float getGreen() const { return green; }
    float getBlue() const { return blue; }

};

#endif // PHYSICS_H