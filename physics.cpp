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
	Data.genShapes(n);

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
		Data.addShape(vertices, (nodes[i] + 1) * 2);
		delete[] vertices;
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
		pendNum[i].setPos(pendNum[i].polarToCartVert());

		if (i)
			pendNum[i].setPivot(pendNum[i - 1].getPos());
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
		// Correctly parenthesized gravity term using 0-based indexing
		B(i) = -grav * (numPend - i) * std::sin(pendNum[i].getAngle());

		for (unsigned int j = 0; j < numPend; j++) {
			// The effective mass term shared by M and the Coriolis/Centrifugal force
			double massFactor = numPend - std::max(i, j);

			// Difference in angles
			double deltaTheta = pendNum[i].getAngle() - pendNum[j].getAngle();

			// Mass matrix M(i, j)
			M(i, j) = massFactor * std::cos(deltaTheta);

			// Coriolis / Centrifugal forces
			B(i) -= massFactor * std::pow(pendNum[j].getAngVel(), 2.0) * std::sin(deltaTheta);
		}
	}

	// Use LLT (Cholesky decomposition) because the Mass matrix is symmetric positive-definite.
	// This is faster and more stable than LU decomposition for N-pendulums.
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

	// Center vertex
	vertices[0] = getPos().x;
	vertices[1] = getPos().y;

	// Circle perimeter vertices
	float angle_offset = 0.0f;
	for (unsigned int i = 0; i <= nodes; i++) {
		vertices[2 * (i + 1)] = getPos().x + radius * std::cos(angle_offset);
		vertices[2 * (i + 1) + 1] = getPos().y + radius * std::sin(angle_offset);
		angle_offset += 2.0f * M_PI / static_cast<float>(nodes);
	}

	// Return the number of vertices to draw (center + perimeter + closing vertex)
	return nodes + 2;
}