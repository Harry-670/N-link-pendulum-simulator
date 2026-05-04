# N-Pendulum Simulator


A real-time simulator for an $n$-link compound pendulum, derived from the Lagrangian and integrated with classical fourth-order Runge–Kutta. The chain is rendered in OpenGL and the kinetic, potential, total, and Lagrangian energies are plotted live so the integrator can be sanity-checked against energy conservation.

## Triple pendulum
<img width="1475" height="863" alt="Animation" src="https://github.com/user-attachments/assets/a03ee9a6-32d1-48da-a44f-946f446498eb" />

## 30 nodes

<img width="802" height="544" alt="30 nodes" src="https://github.com/user-attachments/assets/df3e3a12-bce5-4bef-8ff5-cba60bfba1aa" />

## Overview

Each link is a point mass on the end of a rigid massless rod. The first rod hangs from a fixed anchor; every subsequent rod hangs from the bob above it, so the chain has $n$ degrees of freedom — one angle per rod. The motion is found by writing the Lagrangian $\mathcal{L} = T - V$, applying the Euler–Lagrange equations, and assembling the result into a linear system $\mathbf{M}(\boldsymbol\theta)\,\ddot{\boldsymbol\theta} = \mathbf{b}(\boldsymbol\theta, \dot{\boldsymbol\theta})$ that is solved every sub-step of the integrator.

For $n \ge 2$ the system is chaotic, so trajectories diverge sharply with small perturbations of the initial conditions. The total energy $E = T + V$, however, remains constant — any drift visible in the live plot is purely numerical and reflects the truncation error of the RK4 scheme.

## Math

### Conventions

| Symbol | Meaning |
|---|---|
| $L_i$ | length of the $i$-th rod |
| $m_i$ | mass of the $i$-th bob (taken as $1$ in the implementation) |
| $\theta_i$ | angle of the $i$-th rod, measured from the $-\hat{y}$ axis, counter-clockwise |
| $\dot\theta_i,\ \ddot\theta_i$ | angular velocity, angular acceleration |
| $g$ | gravitational acceleration ($9.81\,\mathrm{m/s^2}$) |
| $M_i$ | cumulative mass at and below joint $i$, i.e. $\sum_{k=i}^{n} m_k$ |

For unit masses, $M_i = n - i + 1$. The implementation works with 0-indexed arrays and uses `numPend - i`, which equals $M_{i+1}$ in the 1-indexed form.

### Cartesian coordinates

Placing the anchor at the origin, the position of the $i$-th bob is

$$x_i = \sum_{j=1}^{i} L_j \sin\theta_j, \qquad y_i = -\sum_{j=1}^{i} L_j \cos\theta_j.$$

Differentiating with respect to time,

$$\dot{x}_i = \sum_{j=1}^{i} L_j \dot\theta_j \cos\theta_j, \qquad \dot{y}_i = \sum_{j=1}^{i} L_j \dot\theta_j \sin\theta_j.$$

### Kinetic energy

Squared speed of bob $i$:

$$|\mathbf{v}_i|^2 = \dot{x}_i^2 + \dot{y}_i^2 = \left(\sum_{j\le i} L_j \dot\theta_j \cos\theta_j\right)^{\!2} + \left(\sum_{j\le i} L_j \dot\theta_j \sin\theta_j\right)^{\!2}.$$

Cross-multiplying and using $\cos\theta_j\cos\theta_k + \sin\theta_j\sin\theta_k = \cos(\theta_j - \theta_k)$:

$$|\mathbf{v}_i|^2 = \sum_{j,k\le i} L_j L_k\, \dot\theta_j \dot\theta_k\, \cos(\theta_j - \theta_k).$$

The total kinetic energy is

$$T = \tfrac{1}{2}\sum_{i=1}^n m_i |\mathbf{v}_i|^2 = \tfrac{1}{2}\sum_{i=1}^n m_i \sum_{j,k\le i} L_j L_k\, \dot\theta_j \dot\theta_k\, \cos(\theta_j - \theta_k).$$

Swap the order of summation. For fixed $j, k$, the inner index $i$ runs over $\max(j,k) \le i \le n$, so $\sum_{i \ge \max(j,k)} m_i = M_{\max(j,k)}$:

$$\boxed{\,T = \tfrac{1}{2} \sum_{j,k=1}^{n} M_{\max(j,k)}\, L_j L_k\, \dot\theta_j \dot\theta_k \cos(\theta_j - \theta_k)\,}$$

This is what `Simulation::calcKinetic` evaluates as a double sum over every pair $(j,k)$.

### Potential energy

Take the anchor as the height reference:

$$V = \sum_{i=1}^n m_i g y_i = -g \sum_{i=1}^n m_i \sum_{j=1}^i L_j \cos\theta_j.$$

Swapping summation order ($i$ now runs $j \le i \le n$):

$$\boxed{\,V = -g \sum_{j=1}^{n} M_j\, L_j \cos\theta_j\,}$$

This is `Simulation::calcPotential`. Note that the absolute value of $V$ depends on where you fix the reference height — only differences in $V$ are physical, so a constant offset cancels out of the dynamics.

### Equations of motion

The Lagrangian is $\mathcal{L} = T - V$. The Euler–Lagrange equations,

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot\theta_i} - \frac{\partial \mathcal{L}}{\partial \theta_i} = 0,$$

require $\partial T/\partial \dot\theta_i$, $\frac{d}{dt}(\partial T/\partial \dot\theta_i)$, $\partial T/\partial \theta_i$, and $\partial V/\partial \theta_i$.

**Generalized momentum.** Differentiating $T$ with respect to $\dot\theta_i$ picks up both the $j=i$ and $k=i$ terms in the double sum, which by symmetry collapse to

$$\frac{\partial T}{\partial \dot\theta_i} = \sum_{j=1}^n M_{\max(i,j)}\, L_i L_j \cos(\theta_i - \theta_j)\,\dot\theta_j.$$

**Time derivative.** Using $\frac{d}{dt}\cos(\theta_i - \theta_j) = -\sin(\theta_i - \theta_j)(\dot\theta_i - \dot\theta_j)$,

$$\frac{d}{dt}\frac{\partial T}{\partial \dot\theta_i} = \sum_j M_{\max(i,j)} L_i L_j \cos(\theta_i - \theta_j)\,\ddot\theta_j \;-\; \sum_j M_{\max(i,j)} L_i L_j \sin(\theta_i - \theta_j)\,\dot\theta_i \dot\theta_j \;+\; \sum_j M_{\max(i,j)} L_i L_j \sin(\theta_i - \theta_j)\,\dot\theta_j^2.$$

**Coordinate derivative.** Only terms in $T$ where $j=i$ or $k=i$ depend on $\theta_i$. Combining both contributions,

$$\frac{\partial T}{\partial \theta_i} = -\sum_j M_{\max(i,j)} L_i L_j \sin(\theta_i - \theta_j)\, \dot\theta_i \dot\theta_j.$$

**Cancellation.** Subtracting, the $\dot\theta_i \dot\theta_j$ cross terms cancel, leaving

$$\frac{d}{dt}\frac{\partial T}{\partial \dot\theta_i} - \frac{\partial T}{\partial \theta_i} = \sum_j M_{\max(i,j)} L_i L_j \cos(\theta_i - \theta_j)\,\ddot\theta_j + \sum_j M_{\max(i,j)} L_i L_j \sin(\theta_i - \theta_j)\,\dot\theta_j^2.$$

**Potential gradient.**

$$\frac{\partial V}{\partial \theta_i} = g M_i L_i \sin\theta_i.$$

**Final equation.** Putting it together and rearranging into matrix–vector form,

$$\boxed{\;\sum_{j=1}^n M_{\max(i,j)} L_i L_j \cos(\theta_i - \theta_j)\, \ddot\theta_j \;=\; -g M_i L_i \sin\theta_i - \sum_{j=1}^n M_{\max(i,j)} L_i L_j \sin(\theta_i - \theta_j)\, \dot\theta_j^2\;}$$

i.e. $\mathbf{M}(\boldsymbol\theta)\,\ddot{\boldsymbol\theta} = \mathbf{b}(\boldsymbol\theta, \dot{\boldsymbol\theta})$ with

$$M_{ij} = M_{\max(i,j)} L_i L_j \cos(\theta_i - \theta_j), \qquad b_i = -g M_i L_i \sin\theta_i - \sum_{j} M_{\max(i,j)} L_i L_j \sin(\theta_i - \theta_j)\,\dot\theta_j^2.$$

The mass matrix is symmetric and, for any physical configuration, positive definite. `Simulation::calcAngAcc` builds $\mathbf{M}$ and $\mathbf{b}$ then calls `Eigen::LLT` (Cholesky), which is both numerically stable and faster than a general solver.

### Time integration

The equations of motion are a first-order ODE in the state $\mathbf{y} = (\boldsymbol\theta, \dot{\boldsymbol\theta})$:

$$\dot{\mathbf{y}} = f(\mathbf{y}) = \begin{pmatrix} \dot{\boldsymbol\theta} \\ \mathbf{M}(\boldsymbol\theta)^{-1}\mathbf{b}(\boldsymbol\theta, \dot{\boldsymbol\theta}) \end{pmatrix}.$$

With step size $h$, classical fourth-order Runge–Kutta is

| Stage | Evaluation |
|------:|:------------|
| $k_1$ | $f(\mathbf{y}_n)$ |
| $k_2$ | $f(\mathbf{y}_n + \tfrac{h}{2}k_1)$ |
| $k_3$ | $f(\mathbf{y}_n + \tfrac{h}{2}k_2)$ |
| $k_4$ | $f(\mathbf{y}_n + h\,k_3)$ |

$$\mathbf{y}_{n+1} = \mathbf{y}_n + \frac{h}{6}\big(k_1 + 2k_2 + 2k_3 + k_4\big).$$

`Simulation::RK4Step` implements this directly: it snapshots the original $(\theta, \dot\theta)$, evaluates each $k_i$ by calling `calcAngAcc()` at the appropriate intermediate state, and writes back the weighted sum. The integrator has local truncation error $O(h^5)$ and global error $O(h^4)$.

The render loop runs as fast as the GPU allows, but the physics step is gated to a fixed cadence (10 ms by default) so the integration is independent of frame rate.

### Energy conservation

The system is autonomous and dissipation-free, so $E = T + V$ is a conserved quantity. The live plot shows $T$, $V$, $E$, and the Lagrangian $\mathcal{L} = T - V$ over a rolling 1.5-second window. With a correctly implemented RK4 and a small enough $h$, the $E$ trace should be visibly flat; significant drift indicates either an integrator bug or a step size too large for the chaotic regime in question.

## Project layout

| File | Contents |
|---|---|
| `Physics.h` / `Physics.cpp` | `Simulation` and `Particle` classes — integrator, mass-matrix assembly, energy functions, polar↔cartesian helpers. |
| `glSetup.h` / `glSetup.cpp` | `DataStore` (window, VAOs/VBOs, vertex buffers), file reader, and shader compile/link. |
| `main.cpp` | Render loop, ImGui/ImPlot setup, energy-plot wiring, simulation parameters. |
| `VertexShader.txt`, `FragmentShader.txt` | GLSL source loaded at runtime. |

## Configuration

The number of links and their physical parameters are set at the top of `main()`:

```cpp
unsigned int n{ 3 };
float* length     = new float[n] { 1.0f, 1.0f, 1.0f };       // rod lengths
float* initAng    = new float[n] { 1.0f, 0.5f, 0.3f };       // initial angles (rad, from -y, CCW)
float* initAngVel = new float[n] { 0.1f, 0.0f, 5.0f };       // initial angular velocities (rad/s)
float* radius     = new float[n] { 0.1f, 0.1f, 0.1f };       // bob radii (visual only)
unsigned int* nodes = new unsigned int[n] { 10, 10, 10 };    // triangle-fan resolution per bob
Sim.setTimeStep(0.01f);                                       // RK4 step size (s)
```

Anchor position, window size, and simulation-space scaling are also set in `main()`.

## Dependencies

- OpenGL 3.3 core + GLAD + GLFW
- [glm](https://github.com/g-truc/glm) — vector and matrix math
- [Eigen 5.0](https://eigen.tuxfamily.org/) — dense linear solver
- [Dear ImGui](https://github.com/ocornut/imgui) + [ImPlot](https://github.com/epezent/implot) — UI and live plots

## Building

The project builds against any C++17 toolchain with the dependencies above on the include path. Both `FragmentShader.txt` and `VertexShader.txt` are loaded relative to the working directory at runtime, so run the binary from the directory containing them.

## References

- Goldstein, *Classical Mechanics*, 3rd ed. — Lagrangian formulation and Euler–Lagrange equations.
- Hairer, Nørsett, Wanner, *Solving Ordinary Differential Equations I*, ch. II — Runge–Kutta methods and order conditions.
