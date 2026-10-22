# Quadrotor GNC Project Design

The intent of this file is to hold the following information:
    
    - Goals: What are the learning goals of this project and what intends to accomplish.
    - Scope: What is within the scope and what is future work.
    - Decision log: A small summary on major decisions.
    - Verification strategy: How the work done will be verified.

## Goals

The primary objective of this project is to solidify core engineering practices and control theory fundamentals:

* Implementing software configurations via automated Continuous Integration (CI) pipelines.
* Utilizing C++20 language features alongside Python modeling tools.
* Developing and tuning cascaded PID/LQR feedback loops.
* Implementing state estimation algorithms for an autonomous system.

## Scope & Non-Goals
* **In Scope:** Pure algorithmic implementation of a 6-DoF quadrotor plant, a cascaded controller, and an Error-State Kalman Filter (ESKF) running on simulation models.
* **Non-Goals / Future Work:** 
  * Direct physical hardware implementation.
  * Simultaneous Localization and Mapping (SLAM).

## Decision Log

### D-001: C++20 Standard Compliance with Extensions Disabled
* **Decision:** Set `CMAKE_CXX_STANDARD 20` and `CMAKE_CXX_EXTENSIONS OFF`.
* **Rationale:** 
  * **Interface Constraints:** Native C++20 **Concepts** enforce strict compile-time type and interface constraints on our template structures, generating clean errors at build time.
  * **Memory Boundaries:** `std::span` offers non-owning, bounded views over contiguous sensor data streams to eliminate manual raw pointer tracking hazards.
  * **Configuration Clarity:** **Designated initializers** ensure that static gain structs are populated by explicit field name definitions, mitigating parameter-swapping risks.
  * **Portability:** Disabling extensions prevents reliance on vendor-specific compiler quirks, ensuring standard compilation across GCC and Clang.
* **Alternatives Considered:** C++17 Standard.

### D-002: Python Decoupled "Oracle" for Verification
* **Decision:** Implement a standalone Python reference script to generate baseline evaluation data.
* **Rationale:** 
  * **Algorithmic Isolation:** Writing algorithms first in an interpreted mathematical environment isolates equation derivation issues from C++ syntax or memory management constraints.
  * **CI Pipeline Integration:** The script outputs numeric state vectors to fixed tracking files. The C++ test framework ingests these files natively, maintaining test independence without binding language runtimes during test execution.
* **Alternatives Considered:** Simultaneous pure C++ validation (rejected due to compounding code bugs with derivation errors), or public flight logs (rejected due to a lack of isolated control inputs).

### D-003: Core-Native 6-DoF Simulator with Python Mirror
* **Decision:** Build our own 6-DoF numerical simulation within the native C++ library core, mirrored exactly in Python for algebraic verification.
* **Rationale:** 
  * **Performance & Speed:** Running batch simulations (such as Monte Carlo sweeps) requires a high execution rate. Heavy 3D engines like Gazebo introduce non-essential graphical and physics overhead.
  * **Timing Repeatability:** A fixed-step fourth-order Runge-Kutta (RK4) integrator inside the core ensures deterministic state updates for verification testing. 
* **Alternatives Considered:** Gazebo/Ignition (deferred to a future middleware phase due to high immediate overhead).

### D-004: Middleware-Agnostic Core Library Architecture
* **Decision:** Isolate the core GNC library (`qgnc_core`) from direct ROS 2 or external communication middleware dependencies.
* **Rationale:** 
  * **Test Isolation:** Decoupling core math from communication protocols allows the GoogleTest executable to verify controllers directly without spawning nodes or middleware executors.
  * **Architectural Boundaries:** If middleware is integrated later, it will function strictly as a separate I/O boundary layer passing data primitives to the core library.
* **Alternatives Considered:** Native ROS 2 package layout (rejected due to dependency coupling).

### D-005: Open-Source Governance via the MIT License
* **Decision:** Publish the project repository under the permissive MIT License.
* **Rationale:** 
  * **Portfolio Visibility:** Eliminates adoption barriers for reviewers or potential collaborators.
  * **Liability Protection:** Explicitly disclaims software warranties, matching safety-critical development standards for exploratory flight code.
* **Alternatives Considered:** GPLv3 (rejected due to restrictive downstream copyleft requirements) or private development.


## Verification Strategy

The verification pipeline combines high-level flight scenarios, independent mathematical sanity checks, and a automated "Golden Data" continuous integration flow.

### 1. Test Scenario Matrix

| Scenario | Target Capabilities Validated |
| :--- | :--- |
| **Hover** | Verifies trim equilibrium and baseline static plant stability. |
| **Attitude / Position Step** | Measures step response parameters (rise time, overshoot) to validate nested loop tuning sequence. |
| **Constant-Velocity Straight Line** | Intentionally checks for expected unobservable states (accelerometer bias and absolute heading drift) to map EKF limitation boundaries. |
| **Figure-Eight Trajectory** | Excites translational and rotational cross-axes to verify the ESKF's capability to bound sensor errors and biases. |
| **Wind Gust** | Evaluates disturbance rejection thresholds and tracking performance limits. |
| **GNSS Dropout (10s)** | Tests inertial dead-reckoning degradation rates and subsequent filter re-convergence properties. |
| **Motor Saturation** | Asserts mixer performance and loop behavior against strict actuator upper limits. |

### 2. Independent Analytical Checks

To prevent shared derivation errors between the verification code and the core library, code executions must satisfy baseline physical laws independently:
* **Kinematics Bounds:** An unactuated, zero-velocity free fall state must match the closed-form equation \(z(t) = \frac{1}{2}gt^2\).
* **Rotational Dynamics:** A constant external torque \(\tau\) around a principal axis where cross-coupling products are zero (\(\omega \times I\omega = 0\), or when initialized from rest \(\omega=0\)) must yield a linear angular acceleration \(\dot{\omega} = I^{-1}\tau\).
* **Attitude Invariant:** The norm of the attitude quaternion vector must equal \(1.0\) at each integration step (\(\Vert{}q\Vert{} = 1.0\)).
* **Filter Convergence:** For a strictly stationary hover state linearized exactly as a Linear Time-Invariant (LTI) system, the steady-state discrete Kalman gain matrix must match the algebraic solution of the Discrete Algebraic Riccati Equation (DARE).

### 3. Snapshot Test Pipeline Flow

1. **Generation:** The Python oracle computes a single algorithmic frame (e.g., one RK4 step) and exports parameters to `reference/golden/*.csv`.
2. **Execution:** The automated C++ pipeline pulls the baseline CSV data files, feeds inputs directly to `qgnc_core`, and checks outcomes via GoogleTest using standard floating-point tolerances.