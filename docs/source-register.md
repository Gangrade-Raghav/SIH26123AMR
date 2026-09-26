# Source Register

## AMR Architecture & Theoretical Literature

The project architecture is grounded in established robotics and multi-agent coordination literature:

1. **Decentralized Task Allocation**:
   - Consensus-Based Bundle Algorithm (CBBA) & Asynchronous CBBA (ACBBA) (Choi et al., 2009; Johnson et al., 2011).
   - Time-windowed bundle allocation and auction mechanisms.

2. **Multi-Agent Path Finding (MAPF)**:
   - Rolling-Horizon Collision Resolution (RHCR) for lifelong windowed planning (Li et al., 2021).
   - Priority-Inheritance Backtracking (PIBT) for fast dynamic conflict resolution (Okumura et al., 2022).
   - Spacetime Reservation Tables with velocity headway safety margins.

3. **Distributed Deadlock & Fault Management**:
   - Wait-For-Graph (WFG) cycle detection and distributed deadlock resolution.
   - Dynamic keep-out zones and heartbeat timeout discrimination.
   - Adaptive compute degradation profiles for resource-constrained robotics nodes.

4. **Middleware & Simulation**:
   - ROS 2 Jazzy Jalisco ecosystem and rclpy execution frameworks.
   - Gazebo Harmonic physics simulation and diff-drive AMR kinematics.
