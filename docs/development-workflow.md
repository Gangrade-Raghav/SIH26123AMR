# Development Workflow

This project follows a rigorous, test-driven engineering workflow designed for research-grade robotics systems.

## Core Methodology

Development is structured around a four-phase disciplined cycle:
- **Explore**: Inspect the system, dependencies, and interfaces before modifying code.
- **Plan**: Define specifications, algorithm criteria, and test vectors.
- **Execute**: Make minimal, coherent, and modular modifications.
- **Verify**: Validate against unit, integration, simulation, and hardware-in-the-loop tests.

## Engineering Disciplines

### Subsystem Division

The codebase is partitioned into distinct modular domains:
- **Simulation Integration**: Gazebo Harmonic world environments, robot SDF/URDF definitions, sensor bridges, and TF transformations.
- **Multi-Agent Path Finding (MAPF)**: Rolling Horizon Collision Resolution (RHCR), Single-Agent A*, Spacetime Reservation Tables, and Priority-Inheritance Backtracking (PIBT).
- **Task Allocation**: Decentralized Consensus-Based Bundle Algorithm (CBBA) bidding, consensus mechanisms, and dynamic task insertion.
- **Fault Resilience**: Heartbeat monitoring, adaptive compute modes, deadlock cycle detection (Wait-For-Graph), and keep-out exclusion zones.
- **Observability**: Real-time telemetry streaming, web consoles, and unbuffered ROS 2 logging.

## Verification Gates

Every pull request or milestone submission must fulfill:
1. Automated unit test suite passing with 0 failures (`pytest`).
2. ROS 2 package build without compilation warnings (`colcon build --symlink-install`).
3. Static code analysis and style conformance (`ament_flake8`, `ament_pep257`).
4. End-to-end multi-robot Gazebo Harmonic simulation validation.

## Experiment Branching

- Use dedicated feature branches for algorithm experimentation.
- Mainline integration branches must always maintain full test pass rates and stable simulation execution.
- Maintain reproducibility logs and empirical benchmark telemetry for all formal experimental runs.
