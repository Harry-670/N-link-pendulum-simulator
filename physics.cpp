#include "Physics.h"
#include <cmath>
#include <iostream>

#define M_PI 3.14159265f


void Simulation::RK4Step(unsigned int index) {
    float dt = timeStep;

    // Grab the current actual state
    float currentAngle = pendNum[index].getAngle();
    float currentVel = pendNum[index].getAngVel();
    float** KVel { new float* [4] }, ** KAcc{ new float* [4] };

    float *origAng{new float [numPend]}, * origVel{ new float[numPend] };

	//fill original arrays with current state
    for (int i{}; i < numPend; i++) {
        origAng[i] = pendNum[i].getAngle();
        origVel[i] = pendNum[i].getAngVel();
	}

    for (int i{} ; i < 4 ; i++){ 

		KVel[i] = new float[numPend] {};
		KAcc[i] = new float[numPend] {};


}

    Eigen::VectorXd A{ calcAngAcc() };

    for (int i{}; i < numPend; i++) {

        KVel[0][i] = currentVel;
        KAcc[0][i] = A(i);

    }
    
	//update angle and angular velocity to the temporary values for the next step
	for (int i{}; i < numPend; i++){
	pendNum[i].setAngle(currentAngle + (0.5f * dt * KVel[0][i]));
    {
    // --- k2: Step halfway into the future using k1 ---
    float k2_angle_temp = currentAngle + (0.5f * dt * k1_vel);
    float k2_vel_temp = currentVel + (0.5f * dt * k1_acc);
    float k2_vel = k2_vel_temp;
    float k2_acc = pendNum[index].calcAngAcc(k2_vel_temp, k2_angle_temp);

    // --- k3: Step halfway into the future using k2 ---
    float k3_angle_temp = currentAngle + (0.5f * dt * k2_vel);
    float k3_vel_temp = currentVel + (0.5f * dt * k2_acc);
    float k3_vel = k3_vel_temp;
    float k3_acc = pendNum[index].calcAngAcc(k3_vel_temp, k3_angle_temp);

    // --- k4: Step a FULL timestep into the future using k3 ---
    float k4_angle_temp = currentAngle + (dt * k3_vel);
    float k4_vel_temp = currentVel + (dt * k3_acc);
    float k4_vel = k4_vel_temp;
    float k4_acc = pendNum[index].calcAngAcc(k4_vel_temp, k4_angle_temp);

    // --- Final Update: Weighted average for BOTH position and velocity ---
    float deltaAngle = (dt / 6.0f) * (k1_vel + 2.0f * k2_vel + 2.0f * k3_vel + k4_vel);
    float deltaVel = (dt / 6.0f) * (k1_acc + 2.0f * k2_acc + 2.0f * k3_acc + k4_acc);

    // Apply the changes
    pendNum[index].setAngle(currentAngle + deltaAngle);
    pendNum[index].setAngVel(currentVel + deltaVel);

    // Update visuals
    pendNum[index].pos = pendNum[index].polarToCartVert();
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