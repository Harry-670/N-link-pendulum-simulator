#include "Physics.h"
#include <cmath>
#include <iostream>

#define M_PI 3.14159265f

void Particle::newStep(float timeStep) {
    angAcc = -grav * std::sin(angle) / length;
    angVel += angAcc * timeStep;
    angle += angVel * timeStep;
    pos = polarToCartVert();
}



glm::vec3 const Particle::polarToCartVert() {

    return glm::vec3(length * std::sin(angle) + pivot.x, -length * std::cos(angle) + pivot.y, 0.0f);


}

// --- Utility ---

float* Particle::initCircle() {
    if (nodes < 3) {
        std::cerr << "Too few nodes to draw a circle" << std::endl;
    }
    float* vertices = new float[2 * (nodes + 2)];

    // Center of the circle is exactly at the origin (0, 0)
    vertices[0] = 0.0f;
    vertices[1] = 0.0f;

    float angle_offset = 0.0f;
    for (unsigned int i = 2; i < (nodes + 2) * 2; i += 2) {
        if (i >= (nodes + 1) * 2) {
            vertices[(nodes + 1) * 2] = vertices[2];
            vertices[(nodes + 1) * 2 + 1] = vertices[3];
        }
        else {
            // Draw relative to origin, scaling by radius
            vertices[i] = radius * std::cos(angle_offset);
            vertices[i + 1] = radius * std::sin(angle_offset);
            angle_offset += 2.0f * M_PI / nodes;
        }
    }
    return vertices;
}