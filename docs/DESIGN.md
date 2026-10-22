# Quadrotor Learning Project

The intent of this file is to hold the following information:
    
    - Goals: What are the learning goals of this project and what intends to accomplish.
    - Scope: What is within the scope and what is future work.
    - Decision log: A small summary on major decisions.
    - Verification strategy: How the work done will be verified.

## Goals

I started this project with the goal of closing gaps on my CV and re-learn core concepts of Control Theory. Therefore, the main goal is to utilize the following skills and tools:

    - Continuous Integration and GitHub
    - Linux operating System and ROS2
    - C++ and Python
    - LQR/PID and cascade loops.

With the goal of creating a control system for a quadrotor.

## Scope

The initial milestone of the project is to create the control system of the quadrotor in C++, with its verification being made using Python and CI capabilities. In a second milestone we will integrate this core into a simulation using ROS2. Finally, if the project develops as expected a physical prototype will be assembled by me.

## Decision Log

### D-001: C++20 Standard Compliance with Extensions Disabled
* **Decision:** Set `CMAKE_CXX_STANDARD 20` and force `CMAKE_CXX_EXTENSIONS OFF`.
* **Rationale:** 
  * **Compile-Time Constraints:** Native C++20 **Concepts** allow us to enforce strict compile-time interface constraints on our controller and estimator templates, catching mathematical structural bugs before compilation completes.
  * **Memory Safety & Performance:** Utilizing `std::span` introduces lightweight, non-owning views over contiguous sensor sample buffers, eliminating pointer-arithmetic hazards without adding runtime overhead.
  * **Configuration Clarity:** **Designated initializers** allow us to initialize tuning matrices and flight configuration blocks by explicit member names, drastically reducing the risk of parameter-swapping errors.
  * **Strict Portability:** Turning compiler extensions off prevents reliance on vendor-specific language quirks (e.g., non-standard GCC extensions), ensuring the core GNC library seamlessly cross-compiles across standard host and target toolchains (GCC, Clang, and embedded ARM compilers).
* **Alternatives Considered:** 
  * *C++17 Standard:* Rejected. While more common in older legacy codebases, it lacks native concepts, forcing convoluted template metaprogramming (SFINAE) and offering weaker compile-time (`constexpr`) computation tools.

### D-002: Python Decoupled "Oracle" for Verification
* **Decision:** Implement a Python-based reference script to generate "Golden Baseline" evaluation datasets.
* **Rationale:** 
  * **Algorithmic Isolation:** Isolating mathematical derivation issues from C++ memory management or syntax complexities speeds up development. Developing the core dynamics and matrix transformations first in a highly dynamic environment ensures the math is verified before translating it to rigid C++.
  * **Automated CI Generation:** The Python oracle acts as the mathematical truth source. It exports deterministic state trajectories to static CSV files, which are committed directly to repository tracking. This allows our C++ CI pipeline to ingest data and test logic via simple file reads without maintaining python bindings during testing.
  * **Leveraging Domain Expertise:** Utilizing established proficiency in Python's analytical ecosystem reduces implementation bottlenecks when sketching early filter and matrix designs.
* **Alternatives Considered:** 
  * *Pure C++ Verification:* Rejected due to the high probability of compounding mathematical derivation errors with programming or memory bugs during simultaneous implementation.
  * *Existing Public Datasets:* Rejected. Pre-existing flight logs lack the clean, deterministic control inputs and matching noise profiles needed to rigorously validate specific custom RK4 and ES-EKF internal states.

### D-003: Standalone Python Simulation for Validation
* **Decision:** We will develop a custom, lightweight 6-DoF numerical simulation in Python rather than utilizing a full 3D physics simulator like Gazebo.
* **Rationale:** 
  * **Scalability:** Rigorous validation of flight software requires Monte Carlo testing (running thousands of randomized scenarios to check EKF convergence and controller stability limits). Gazebo is computationally heavy, relies on graphical/physics engine overhead, and runs too slowly for high-throughput batch testing.
  * **Determinism:** A custom Python script using a fixed-step RK4 integrator provides 100% deterministic, repeatable physics. Gazebo's real-time factor can vary, introducing nondeterministic timing anomalies that complicate regression and unit testing.
  * **Industry Practice:** Separating pure algorithmic validation (via fast, high-rate math simulations) from full 3D hardware-in-the-loop environments mimics commercial aerospace validation pipelines.
* **Alternatives Considered:** 
  * *Gazebo / Ignition:* Rejected due to high computational overhead, slower execution speeds, and unnecessary complexity for core GNC algorithm verification.

### D-004: Middleware-Agnostic Core Library Architecture
* **Decision:** Keep the core Guidance, Navigation, and Control library (`qgnc_core`) completely free of ROS 2 or any other communication middleware dependencies.
* **Rationale:** 
  * **Strict Architectural Decoupling:** Separating pure mathematical algorithms (physics, control loop, and error-state estimation) from communication middleware prevents API entanglement and keeps the core codebase highly focused and clean.
  * **Rapid and Lightweight Unit Testing:** Eliminating middleware dependencies allows the GoogleTest suite to execute natively in fractions of a second. Testing core blocks (like the RK4 integrator) does not require spinning up heavy, fragile ROS 2 nodes, executors, or simulated communication layers.
  * **Cross-Hardware Portability:** A pure C++20 library can be seamlessly cross-compiled and deployed across completely different execution layers. It can run inside an agile Python/C++ desktop simulation, scale into a high-fidelity hardware-in-the-loop (HIL) environment, or deploy onto a bare-metal microcontroller (e.g., an STM32 running FreeRTOS) that cannot support a heavy ROS 2 stack.
  * **Boundary Definition for Future Integration:** If ROS 2 is introduced later, it will serve strictly as a thin I/O wrapper. The ROS 2 nodes will handle topic serialization, pass raw data primitives or arrays into `qgnc_core`, and immediately publish the calculated controller outputs.
* **Alternatives Considered:** 
  * *Native ROS 2 Package Integration:* Rejected. Direct integration binds the mathematics to the lifecycles, message formats, and performance overhead of specific middleware versions, severely reducing portability and complicating structural unit testing.

## Verification Strategy

## Verification Strategy

The verification pipeline combines high-level flight scenarios, independent mathematical sanity checks, and a automated "Golden Data" continuous integration flow.

### 1. Test Scenario Matrix

| Scenario | What It Proves / Validates |
| :--- | :--- |
| **Hover** | System trim and static stability; ensures physics equilibrium holds. |
| **Attitude / Position Step** | Evaluates rise time and overshoot; verifies nested control loops are correctly tuned from inner (rate) to outer (position) layers. |
| **Figure-Eight Trajectory** | Excites all translational and rotational axes simultaneously, providing the dynamic acceleration needed to bound ES-EKF states and biases. |
| **Wind Gust** | Evaluates disturbance rejection and control loop robustness against external forces. |
| **GNSS Dropout (10s)** | Tests inertial navigation system (INS) dead-reckoning drift and verifies ES-EKF convergence and recovery when signals return. |
| **Motor Saturation** | Validates the motor mixer limits and actuator anti-windup performance under maximum load. |

### 2. Independent Analytical Checks

To prevent shared mathematical derivation bugs from passing unnoticed across both the Python oracle and C++ libraries, the system must satisfy these standalone physics rules without relying on an external reference code block:
* **Kinematics Bounds:** A drone in unactuated free fall must strictly follow the equation $z = \frac{1}{2}gt^2$.
* **Rotational Dynamics:** Applying a constant torque input must yield a perfectly linear, constant angular acceleration.
* **Attitude Constraints:** The magnitude/norm of the unit quaternion state vector must remain exactly equal to $1.0$ at all times ($\Vert{}q\Vert{} = 1$).
* **Filter Convergence:** The estimator's steady-state Kalman gain matrix must match the algebraic solution calculated via `scipy.linalg.solve_discrete_are`.

### 3. Golden Test Pipeline Flow

We use an automated snapshot testing pattern to lock in mathematical behaviors across languages without coupling testing environments:

1. **Oracle Generation:** The Python simulation runs a deterministic slice of time (e.g., a single RK4 integration step, a single filter measurement update, or a micro-trajectory) and exports the raw inputs and calculated outputs to `reference/golden/*.csv`.
2. **Git Tracking:** These static tracking sheets are committed directly into Git repository storage, decoupling our testing framework from heavy analytical runtime dependencies.
3. **C++ Unit Assertion:** A GoogleTest routine loads the target CSV dataset on the continuous integration (CI) platform, passes the inputs directly to the `qgnc_core` routines, and compares the outputs using floating-point tolerances (typically `1e-9` for isolated steps; expanded for multi-step trajectories to absorb numerical accumulation limits).

### 4. Hardware-Data Stretch Goal
Once primary validation passes against the deterministic simulation, we will run the ES-EKF against the open-source **EuRoC MAV dataset** to verify performance against non-ideal, noisy real-world sensor streams.
