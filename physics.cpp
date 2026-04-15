#include "Physics.h"
#include <cmath>
#include <iostream>

#define M_PI 3.14159265f

// Simulation setup
DataStore Simulation::setUpPend(unsigned int n, glm::vec3 pivot, float* length, float* initAng, float* initAngVel, unsigned int* nodes) {

	GLfloat screenX{ 800 }, screenY{ 800 };
	DataStore Data{};

	GLFWwindow* window{ Data.glSetupWindow(screenX, screenY) };
	pendNum = new Particle[n]{};
	numPend = n;
	Data.genShapes(n*2);

    // Track the previous particle's position (start at the anchor pivot)
	glm::vec3 prevPos = pivot;
	for (unsigned int i{}; i < n; i++) {
		float* vertices;

		pendNum[i].setAngVel(initAngVel[i]);
		pendNum[i].setAngle(initAng[i]);
		pendNum[i].setNodes(nodes[i]);
		pendNum[i].setLength(length[i]);
		pendNum[i].setRadius(0.1f);

		// Set this particle's pivot to the previous particle's absolute position
		pendNum[i].setPivot(prevPos);

		// Compute absolute position from pivot + polar offset
		pendNum[i].setPos(pendNum[i].polarToCartVert());

		// Next particle will pivot about this particle's position
		prevPos = pendNum[i].getPos();
		pendNum[i].initCircle(vertices);
		Data.addShape(vertices, nodes[i] + 2);

		// Create line from pivot to bob (0,0) to (0,length)
		float* lineVertices{ new float[4] {0.0f, 0.0f, 0.0f, -length[i]} };
		Data.addShape(lineVertices, 2);
	}

	return Data;
}

// Physics simulation step
void Simulation::RK4Step() {
	float dt = timeStep;
	float** KVel{ new float* [4] }, ** KAcc{ new float* [4] };
	float* origAng{ new float[numPend] }, * origAngVel{ new float[numPend] };

	// Store original state
	for (unsigned int i{}; i < numPend; i++) {
		origAng[i] = pendNum[i].getAngle();
		origAngVel[i] = pendNum[i].getAngVel();
	}

	// Initialize K arrays
	for (unsigned int i{}; i < 4; i++) {
		KVel[i] = new float[numPend]{};
		KAcc[i] = new float[numPend]{};
	}

	// RK4 step 1
	Eigen::VectorXd A{ calcAngAcc() };
	for (unsigned int i{}; i < numPend; i++) {
		KVel[0][i] = origAngVel[i];
		KAcc[0][i] = A(i);
		pendNum[i].setAngle(origAng[i] + (0.5f * dt * KVel[0][i]));
		pendNum[i].setAngVel(origAngVel[i] + (0.5f * dt * KAcc[0][i]));
	}

	// RK4 step 2
	A = calcAngAcc();
	for (unsigned int i{}; i < numPend; i++) {
		KVel[1][i] = pendNum[i].getAngVel();
		KAcc[1][i] = A(i);
		pendNum[i].setAngle(origAng[i] + (0.5f * dt * KVel[1][i]));
		pendNum[i].setAngVel(origAngVel[i] + (0.5f * dt * KAcc[1][i]));
	}

	// RK4 step 3
	A = calcAngAcc();
	for (unsigned int i{}; i < numPend; i++) {
		KVel[2][i] = pendNum[i].getAngVel();
		KAcc[2][i] = A(i);
		pendNum[i].setAngle(origAng[i] + (0.5f * dt * KVel[2][i]));
		pendNum[i].setAngVel(origAngVel[i] + (0.5f * dt * KAcc[2][i]));
	}

	// RK4 step 4
	A = calcAngAcc();
	for (unsigned int i{}; i < numPend; i++) {
		KVel[3][i] = pendNum[i].getAngVel();
		KAcc[3][i] = A(i);
	}

	// Apply weighted average and update
	for (unsigned int i{}; i < numPend; i++) {
		pendNum[i].setAngle(origAng[i] + (dt / 6.0f) * (KVel[0][i] + 2.0f * KVel[1][i] + 2.0f * KVel[2][i] + KVel[3][i]));
		pendNum[i].setAngVel(origAngVel[i] + (dt / 6.0f) * (KAcc[0][i] + 2.0f * KAcc[1][i] + 2.0f * KAcc[2][i] + KAcc[3][i]));
		pendNum[i].setAngPrev(origAng[i]);
	}

	// Update positions
	for (unsigned int i{}; i < numPend; i++) {

		pendNum[i].setPrevPos(pendNum[i].getPos());
		if (i)
			pendNum[i].setPivot(pendNum[i - 1].getPos());

		pendNum[i].setPos(pendNum[i].polarToCartVert());

	}

	// Cleanup
	for (unsigned int i{}; i < 4; i++) {
		delete[] KVel[i];
		delete[] KAcc[i];
	}
	delete[] KVel;
	delete[] KAcc;
	delete[] origAng;
	delete[] origAngVel;
}

// Angular acceleration calculation
Eigen::VectorXd Simulation::calcAngAcc() {
	Eigen::MatrixXd M(numPend, numPend);
	Eigen::VectorXd B(numPend);

	for (unsigned int i = 0; i < numPend; i++) {
		float Li = pendNum[i].getLength();
		float theta_i = pendNum[i].getAngle();

		// Gravity term: -g * L_i * (number of masses below) * sin(theta_i)
		// Note: (numPend - i) represents the "effective mass factor"
		B(i) = -grav * Li * (numPend - i) * std::sin(theta_i);

		for (unsigned int j = 0; j < numPend; j++) {
			float Lj = pendNum[j].getLength();
			float theta_j = pendNum[j].getAngle();
			float omega_j = pendNum[j].getAngVel();

			double massFactor = numPend - std::max(i, j);
			double deltaTheta = theta_i - theta_j;

			// Mass matrix scale: Li * Lj * massFactor
			M(i, j) = massFactor * Li * Lj * std::cos(deltaTheta);

			// Coriolis/Centrifugal scale: Li * Lj * massFactor * omega^2
			B(i) -= massFactor * Li * Lj * std::pow(omega_j, 2.0) * std::sin(deltaTheta);
		}
	}

	return M.llt().solve(B);
}

// Particle geometry
glm::vec3 const Particle::polarToCartVert() {
	return glm::vec3(length * std::sin(angle) + pivot.x, -length * std::cos(angle) + pivot.y, 0.0f);
}

unsigned int Particle::initCircle(float*& vertices) {
	if (nodes < 3) {
		std::cerr << "Too few nodes to draw a circle" << std::endl;
	}
	vertices = new float[2 * (nodes + 2)];

	// Center vertex should be at the local origin, NOT getPos()
	vertices[0] = 0.0f;
	vertices[1] = 0.0f;

	// Circle perimeter vertices
	float angle_offset = 0.0f;
	for (unsigned int i = 0; i <= nodes; i++) {
		vertices[2 * (i + 1)] = radius * std::cos(angle_offset);
		vertices[2 * (i + 1) + 1] = radius * std::sin(angle_offset);
		angle_offset += 2.0f * M_PI / static_cast<float>(nodes);
	}

	return nodes + 2;

}