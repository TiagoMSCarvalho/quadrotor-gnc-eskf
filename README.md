# Quadrotor GNC Simulator & Estimator (`quadrotor-gnc-eskf`)

**Work in Progress**

A modular Guidance, Navigation, and Control (GNC) project focusing on a 6-DoF quadrotor simulation environment. The architecture implements a core-native physics simulation, multi-loop flight controllers, and an Error-State Kalman Filter (ESKF).

## Planned Validation Scenarios
The implementation is progressively verified through automated snapshot testing using a Python math oracle against the following profiles:
* Static Trim & Hover Stability
* Cascaded Position & Attitude Step Responses
* Constant-Velocity Trajectory Drift Profiling
* Multi-Axis Dynamic Figure-Eight Tracking
* Disturbance Rejection & Wind Gust Recovery
* GPS-Denied Dead-Reckoning Navigation

## Documentation
For complete architectural details, language specifications, and the verification framework log, see the [Project Design Document](docs/DESIGN.md).

## License
Distributed under the permissive MIT License. See `LICENSE` for details.
