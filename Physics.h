#ifndef PHYSICS_H
#define PHYSICS_H

#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <eigen-5.0.0/Eigen/dense>
#include <algorithm>
#include "glSetup.h"

#define grav 1.0f

class Particle;

class Simulation {
private:
    DataStore Data{};
    Particle* pendNum;
    float timeStep;
    unsigned int numPend;
public:
    Simulation() : Data{}, pendNum(nullptr), timeStep(0.0f), numPend(0) {}

    Particle* getPendNum() { return pendNum; }
    float getTimeStep() const { return timeStep; }
    unsigned int getNumPend() const { return numPend; }
    DataStore& getDataStore() { return Data; }

    void setTimeStep(float newTimeStep) { timeStep = newTimeStep; }
    void setNumPend(unsigned int newNumPend) { numPend = newNumPend; }

    DataStore setUpPend(unsigned int n, glm::vec3 pivot, float* length, float* initAng, float* initAngVel, unsigned int* nodes);
    void RK4Step();
    Eigen::VectorXd calcAngAcc();
};

class Particle {
private:
    // Position vectors
    glm::vec3 pos, posPrev, vel, acc, pivot;

    // Angular properties
    float angle, angPrev, angVel, angAcc, delAng, delAngVel;

    // Geometry properties
    float radius, length;
    unsigned int nodes;

    // Color properties
    float red, green, blue;

public:
    // Constructors
    Particle(glm::vec3 pos, glm::vec3 posPrev, glm::vec3 vel, glm::vec3 acc, glm::vec3 pivot,
             float angle, float angPrev, float angVel, float angAcc,
             float radius, float length, unsigned int nodes,
             float red, float green, float blue)
        : pos(pos), posPrev(posPrev), vel(vel), acc(acc), pivot(pivot),
          angle(angle), angPrev(angPrev), angVel(angVel), angAcc(angAcc), delAng(0.0f), delAngVel(0.0f),
          radius(radius), length(length), nodes(nodes),
          red(red), green(green), blue(blue) {
    }

    Particle() : Particle(glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f), glm::vec3(0.0f),
                          0.0f, 0.0f, 0.0f, 0.0f,
                          0.0f, 0.0f, 0,
                          0.0f, 0.0f, 0.0f) {
    }

    // Physics Logic
    unsigned int initCircle(float*& vertices);

    glm::vec3 const polarToCartVert();

    // Setters
    void setPos(glm::vec3 newPos) { pos = newPos; }
    void setPrevPos(glm::vec3 newPosPrev) { posPrev = newPosPrev; }
    void setPivot(glm::vec3 newPivot) { pivot = newPivot; }
    void setPivot(float x, float y) { pivot = glm::vec3(x, y, 0.0f); }
    void setVel(glm::vec3 newVel) { vel = newVel; }
    void setAcc(glm::vec3 newAcc) { acc = newAcc; }
    void setAngle(float angle) { this->angle = angle; }
    void setAngPrev(float angPrev) { this->angPrev = angPrev; }
    void setAngVel(float angVel) { this->angVel = angVel; }
    void setAngAcc(float angAcc) { this->angAcc = angAcc; }
    void setDelAng(float delAng) { this->delAng = delAng; }
    void setDelAngVel(float delAngVel) { this->delAngVel = delAngVel; }
    void setRadius(float radius) { this->radius = radius; }
    void setLength(float length) { this->length = length; }
    void setNodes(unsigned int nodes) { this->nodes = nodes; }
    void setRed(float r) { red = r; }
    void setGreen(float g) { green = g; }
    void setBlue(float b) { blue = b; }

    // Getters
    glm::vec3 getPos() const { return pos; }
    glm::vec3 getPrevPos() const { return posPrev; }
    glm::vec3 getPivot() const { return pivot; }
    glm::vec3 getVel() const { return vel; }
    glm::vec3 getAcc() const { return acc; }
    float getAngle() const { return angle; }
    float getAngPrev() const { return angPrev; }
    float getAngVel() const { return angVel; }
    float getAngAcc() const { return angAcc; }
    float getDelAng() const { return delAng; }
    float getDelAngVel() const { return delAngVel; }
    float getRad() const { return radius; }
    float getLength() const { return length; }
    unsigned int getNodes() const { return nodes; }
    unsigned int getCircleVertices() const { return nodes + 2; }
    unsigned int getLineVertices() const { return 2; }
    float getRed() const { return red; }
    float getGreen() const { return green; }
    float getBlue() const { return blue; }
};

#endif // PHYSICS_H