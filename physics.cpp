#include "Physics.h"
#define grav 9.81
#define M_PI 3.14159265f

// Builds the chain: each particle's pivot is the previous particle's
// world-space position. The first particle pivots about the supplied anchor.
DataStore Simulation::setUpPend(unsigned int n, glm::vec3 pivot,
    float* length, float* initAng, float* initAngVel,
    float* radius, unsigned int* nodes,
    GLuint windowX, GLuint windowY,
    GLuint simX, GLuint simY) {
    DataStore Data{};
    Data.glSetupWindow(windowX, windowY);

    pendArray = new Particle[n]{};
    numPend = n;

    // Two shapes per link: one circle for the bob, one line for the rod.
    Data.genShapes(n * 2);

    glm::vec3 prevPos = pivot;
    for (unsigned int i = 0; i < n; i++) {
        pendArray[i].setAngVel(initAngVel[i]);
        pendArray[i].setAngle(initAng[i]);
        pendArray[i].setNodes(nodes[i]);
        pendArray[i].setLength(length[i]);
        pendArray[i].setRadius(radius[i]);
        pendArray[i].setPivot(prevPos);
        pendArray[i].setPos(pendArray[i].polarToCartVert());
        prevPos = pendArray[i].getPos();

        float* vertices;
        pendArray[i].initCircle(vertices);
        Data.addShape(vertices, nodes[i] + 2);

        // Rod modelled in local space from origin straight down. The model
        // matrix in main rotates and translates it into world space
        float* lineVertices{ new float[4] {0.0f, 0.0f, 0.0f, -length[i]} };
        Data.addShape(lineVertices, 2);
    }

    return Data;
}

// fourth-order Runge-Kutta on the angular state (angle, angVel)
// k1 is evaluated at the original state, k2 and k3 at half-step states using
// the previous k, and k4 at the full-step state. The new state is the
// weighted average (k1 + 2k2 + 2k3 + k4) / 6
void Simulation::RK4Step() {
    const float dt = timeStep;

    float** KVel = new float* [4];
    float** KAcc = new float* [4];
    for (unsigned int i = 0; i < 4; i++) {
        KVel[i] = new float[numPend] {};
        KAcc[i] = new float[numPend] {};
    }

    // Snapshot of the state at the start of the step
    float* origAng = new float[numPend];
    float* origAngVel = new float[numPend];
    for (unsigned int i = 0; i < numPend; i++) {
        origAng[i] = pendArray[i].getAngle();
        origAngVel[i] = pendArray[i].getAngVel();
    }

    // k1: evaluated at the original state
    Eigen::VectorXd A = calcAngAcc();
    for (unsigned int i = 0; i < numPend; i++) {
        KVel[0][i] = origAngVel[i];
        KAcc[0][i] = static_cast<float>(A(i));
        pendArray[i].setAngle(origAng[i] + 0.5f * dt * KVel[0][i]);
        pendArray[i].setAngVel(origAngVel[i] + 0.5f * dt * KAcc[0][i]);
    }

    // k2: evaluated at the half-step state advanced by k1
    A = calcAngAcc();
    for (unsigned int i = 0; i < numPend; i++) {
        KVel[1][i] = pendArray[i].getAngVel();
        KAcc[1][i] = static_cast<float>(A(i));
        pendArray[i].setAngle(origAng[i] + 0.5f * dt * KVel[1][i]);
        pendArray[i].setAngVel(origAngVel[i] + 0.5f * dt * KAcc[1][i]);
    }

    // k3: evaluated at the half-step state advanced by k2
    A = calcAngAcc();
    for (unsigned int i = 0; i < numPend; i++) {
        KVel[2][i] = pendArray[i].getAngVel();
        KAcc[2][i] = static_cast<float>(A(i));
        pendArray[i].setAngle(origAng[i] + dt * KVel[2][i]);
        pendArray[i].setAngVel(origAngVel[i] + dt * KAcc[2][i]);
    }

    // k4: evaluated at the full-step state advanced by k3
    A = calcAngAcc();
    for (unsigned int i = 0; i < numPend; i++) {
        KVel[3][i] = pendArray[i].getAngVel();
        KAcc[3][i] = static_cast<float>(A(i));
    }

    // Weighted average -> new state
    for (unsigned int i = 0; i < numPend; i++) {
        pendArray[i].setAngle(origAng[i] + (dt / 6.0f) * (KVel[0][i] + 2.0f * KVel[1][i] + 2.0f * KVel[2][i] + KVel[3][i]));
        pendArray[i].setAngVel(origAngVel[i] + (dt / 6.0f) * (KAcc[0][i] + 2.0f * KAcc[1][i] + 2.0f * KAcc[2][i] + KAcc[3][i]));
        pendArray[i].setAngPrev(origAng[i]);
    }

    // Refresh cartesian pivots and positions from the new angular state
    for (unsigned int i = 0; i < numPend; i++) {
        pendArray[i].setPrevPos(pendArray[i].getPos());
        if (i)
            pendArray[i].setPivot(pendArray[i - 1].getPos());
        pendArray[i].setPos(pendArray[i].polarToCartVert());
    }

    for (unsigned int i = 0; i < 4; i++) {
        delete[] KVel[i];
        delete[] KAcc[i];
    }
    delete[] KVel;
    delete[] KAcc;
    delete[] origAng;
    delete[] origAngVel;
}

// Solves M(theta) * theta_doubledot = b(theta, theta_dot) for the angular
// accelerations. Equation derived from the Lagrangian of n unit-mass links
Eigen::VectorXd Simulation::calcAngAcc() {
    Eigen::MatrixXd M(numPend, numPend);
    Eigen::VectorXd B(numPend);

    for (unsigned int i = 0; i < numPend; i++) {
        const float Li = pendArray[i].getLength();
        const float theta_i = pendArray[i].getAngle();

        // Gravity term: -g * L_i * M_i * sin(theta_i), where M_i = numPend - i
        // is the total (unit) mass hanging from this joint and below
        B(i) = -grav * Li * (numPend - i) * std::sin(theta_i);

        for (unsigned int j = 0; j < numPend; j++) {
            const float Lj = pendArray[j].getLength();
            const float theta_j = pendArray[j].getAngle();
            const float omega_j = pendArray[j].getAngVel();

            const double massFactor = numPend - std::max(i, j);
            const double deltaTheta = theta_i - theta_j;

            // Mass matrix entry.
            M(i, j) = massFactor * Li * Lj * std::cos(deltaTheta);

            // Centrifugal/Coriolis contribution to b
            B(i) -= massFactor * Li * Lj * std::pow(omega_j, 2.0) * std::sin(deltaTheta);
        }
    }

    // M is symmetric positive-definite for a physical pendulum, so Cholesky
    // is both stable and faster than a general solver here
    return M.llt().solve(B);
}

// Polar offset (length, angle) -> world-space position relative to pivot.
// Angle is measured from -y, counter-clockwise
glm::vec3 const Particle::polarToCartVert() {
    return glm::vec3(length * std::sin(angle) + pivot.x,
        -length * std::cos(angle) + pivot.y,
        0.0f);
}

// Total kinetic energy:
//   T = (1/2) * sum_{i,j} M_max(i,j) * L_i * L_j * w_i * w_j * cos(theta_i - theta_j)
float Simulation::calcKinetic() {
    float T {0.0f}, Li, wi, thetai, Lj, wj, thetaj, massFactor;
    for (unsigned int i = 0; i < numPend; i++) {
        Li = pendArray[i].getLength();
        wi = pendArray[i].getAngVel();
        thetai = pendArray[i].getAngle();
        for (unsigned int j = 0; j < numPend; j++) {
            Lj = pendArray[j].getLength();
            wj = pendArray[j].getAngVel();
            thetaj = pendArray[j].getAngle();
            massFactor = static_cast<float>(numPend - std::max(i, j));
            T += massFactor * Li * Lj * wi * wj * std::cos(thetai - thetaj);
        }
    }
    return 0.5f * T;
}

// Total potential energy, taking the anchor as the reference height:
//   V = -g * sum_i M_i * L_i * cos(theta_i)
float Simulation::calcPotential() {
    float V = 0.0f, massFactor;
    for (unsigned int i = 0; i < numPend; i++) {
        massFactor = static_cast<float>(numPend - i);
        V -= grav * massFactor * pendArray[i].getLength() * std::cos(pendArray[i].getAngle());
    }
    return V;
}

// Scales an interleaved (x, y) vertex array into normalized device coords
void normalize(float*& vertices, std::size_t size, GLuint simX, GLuint simY) {
    for (std::size_t i = 0; i < size; i += 2) {
        vertices[i] /= simX;
        vertices[i + 1] /= simY;
    }
}

// Triangle-fan vertex layout: index 0 is the centre, indices 1..nodes+1 walk
// the perimeter (with index nodes+1 closing back onto index 1)
unsigned int Particle::initCircle(float*& vertices) {
    if (nodes < 3) {
        std::cerr << "Too few nodes to draw a circle" << std::endl;
    }
    vertices = new float[2 * (nodes + 2)];

    vertices[0] = 0.0f;
    vertices[1] = 0.0f;

    float angle_offset = 0.0f;
    for (unsigned int i = 0; i <= nodes; i++) {
        vertices[2 * (i + 1)] = radius * std::cos(angle_offset);
        vertices[2 * (i + 1) + 1] = radius * std::sin(angle_offset);
        angle_offset += 2.0f * M_PI / static_cast<float>(nodes);
    }

    return nodes + 2;
}