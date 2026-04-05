#include "Physics.h"
#include <cmath>
#include <iostream>

#define M_PI 3.14159265f

void Simulation::setUpPend(unsigned int n, float* pivot, float* initAng, float* initAngVel) {

    DataStore Data{};
    Data.genShapes(n);
    
    float* tempPivot{ new float[2] {pivot[0], pivot[1]} };

    Data.addShape(tempPivot, unsigned int size)

}

void Simulation::RK4Step(unsigned int index) {
    float dt = timeStep;

    float** KVel { new float* [4] }, ** KAcc{ new float* [4] };

    float *origAng{new float [numPend]}, * origAngVel{ new float[numPend] };

	//fill original arrays with current state
    for (int i{}; i < numPend; i++) {
        origAng[i] = pendNum[i].getAngle();
        origAngVel[i] = pendNum[i].getAngVel();
	}

    for (int i{} ; i < 4 ; i++){ 

		KVel[i] = new float[numPend] {};
		KAcc[i] = new float[numPend] {};


}

    Eigen::VectorXd A{ calcAngAcc() };

    for (int i{}; i < numPend; i++) {

        KVel[0][i] = origAngVel[i];
        KAcc[0][i] = A(i);
        pendNum[i].setAngle(origAng[i] + (0.5f * dt * KVel[0][i]));
        pendNum[i].setAngVel(origAngVel[i] + (0.5f * dt * KAcc[0][i]));

    }
    
    A = calcAngAcc();

	
    for (int i{}; i < numPend; i++) {

        KVel[1][i] = pendNum[i].getAngVel();
        KAcc[1][i] = A(i);
        pendNum[i].setAngle(origAng[i] + (0.5f * dt * KVel[1][i]));
        pendNum[i].setAngVel(origAngVel[i] + (0.5f * dt * KAcc[1][i]));

    }

    A = calcAngAcc();

    for (int i{}; i < numPend; i++) {

        KVel[2][i] = pendNum[i].getAngVel();
        KAcc[2][i] = A(i);
        pendNum[i].setAngle(origAng[i] + (0.5f * dt * KVel[2][i]));
        pendNum[i].setAngVel(origAngVel[i] + (0.5f * dt * KAcc[2][i]));

    }

    A = calcAngAcc();

    for (int i{}; i < numPend; i++) {

        KVel[3][i] = pendNum[i].getAngVel();
        KAcc[3][i] = A(i);

    }

    //fill the orginals back up and fill prev ang
        // --- Final Update: Weighted average for BOTH position and velocity ---
    for (int i{}; i < numPend; i++) {

        pendNum[i].setAngle(origAng[i]+ (dt / 6.0f) * (KVel[0][i] + 2.0f * KVel[1][i] + 2.0f * KVel[2][i] + KVel[3][i]));
        pendNum[i].setAngVel(origAngVel[i] + (dt / 6.0f) * (KAcc[0][i] + 2.0f * KAcc[1][i] + 2.0f * KAcc[2][i] + KAcc[3][i]));
        pendNum[i].prevAng(origAng[i]);
    }

    // --- Final Update: Weighted average for BOTH position and velocity ---


    // Update visuals
    for (unsigned int i{}; i < numPend; i++) {

    pendNum[i].setPrevPos() = pendNum[i].getPos();
    pendNum[i].setPos() = pendNum[i].polarToCartVert();
    

    }

}




glm::vec3 const Particle::polarToCartVert() {

    return glm::vec3(length * std::sin(angle) + pivot.x, -length * std::cos(angle) + pivot.y, 0.0f);


}
Eigen::VectorXd Simulation::calcAngAcc() {

    Eigen::MatrixXd M(numPend,numPend);
    Eigen::VectorXd B(numPend);

    for (int i{}; i < numPend; i++) {
        B(i) = -grav * (numPend - i + 1) * std::sin(pendNum[i].getAngVel());
        for (int j{}; j < numPend; j++ ) {

            M(i, j) = (numPend - std::max(i, j) + 1) * std::cos(pendNum[i].getAngVel() - pendNum[j].getAngVel());
            B(i) -= (numPend - std::max(i, j) + 1)* std::pow(pendNum[j].getAngVel(),2)* std::sin(pendNum[i].getAngVel() - pendNum[j].getAngVel());

        }

    }

    return M.lu().solve(B);

}
// --- Utility ---

unsigned int Particle::initCircle(float* &vertices ) {
    if (nodes < 3) {
        std::cerr << "Too few nodes to draw a circle" << std::endl;
    }
    vertices = new float[2 * (nodes + 2)];
    unsigned int size{};

    // Center of the circle is exactly at the origin (0, 0)
    vertices[0] = 0.0f;
    vertices[1] = 0.0f;
    size++;
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
        size++;
    }
    return size;
}