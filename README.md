# N-Link Pendulum Simulation

A high-performance, real-time physics simulation of an N-link pendulum system. This project utilizes **Lagrangian mechanics** to derive equations of motion and implements a **4th-order Runge-Kutta (RK4)** integrator to solve the resulting differential equations with high numerical stability.

##  Overview
This project demonstrates the intersection of computational physics, linear algebra, and modern graphics programming. By solving the system of equations $M\ddot{\theta} = B$, it accurately captures the chaotic behavior of multi-link pendulums.

##  Core Features
* **Dynamic N-Link Support:** Simulates any number of connected pendulums, from a simple single swing to the high-chaos behavior of double or triple systems.
* **High-Fidelity Integration:** Uses a **4th-order Runge-Kutta (RK4)** solver instead of simpler Euler methods, ensuring energy conservation and long-term numerical stability.
* **Generalized Physics Engine:** Solves for angular accelerations using **Eigen’s Cholesky decomposition (LLT)** to efficiently handle the symmetric positive-definite mass matrix.
* **Modern OpenGL Pipeline:** Renders via OpenGL 3.3 (Core Profile) using custom GLSL shaders, Vertex Array Objects (VAOs), and dynamic transformation matrices.

##  Tech Stack
* **Language:** C++17
* **Physics/Math:**
    * **Eigen:** For matrix operations and solving the system of equations.
    * **GLM:** For OpenGL-specific matrix transformations and vector math.
* **Graphics:**
    * **OpenGL 3.3:** Core Profile rendering.
    * **GLFW:** Windowing and input handling.
    * **GLAD:** OpenGL function loader.
* **Shaders:** Custom GLSL Vertex and Fragment shaders.

##  How It Works
1.  **Lagrangian Dynamics:** The simulation calculates the **Mass Matrix ($M$)** and the **Force Vector ($B$)** based on the current state of all links (angles, velocities, and lengths).
2.  **Numerical Integration:** The RK4 solver takes four "probes" into the future per timestep to calculate a weighted average of the state change, minimizing error accumulation.
3.  **Rendering:** The physics state is mapped to the GPU. Each link is treated as a local coordinate system, rotated and translated into world space via transformation matrices passed as uniforms to the shaders.

##  Project Structure
* `Physics.cpp/h`: Core simulation logic, RK4 implementation, and Lagrangian math.
* `glSetup.cpp/h`: OpenGL initialization and Vertex Buffer management.
* `main.cpp`: Main simulation loop and rendering logic.
* `shaders/`: GLSL source code for vertex and fragment processing.
