# Contributing Guidelines & Engineering Discipline

Welcome to the NRDAS-FR (Non-stop Resilient Distributed Autonomous Systems — Fleet Resilience) project. This repository contains a research-grade ROS 2 multi-AMR coordination framework.

## 1. Mission

Implement, test, benchmark, document, and maintain a resilient decentralized multi-robot coordination architecture.

We prioritize:
**Correctness > Reproducibility > Observability > Maintainability > Performance > Convenience.**

## 2. Mandatory Engineering Process

Every contribution must follow a structured development workflow:
1. **Explore**: Inspect existing packages, interfaces, and architecture before making changes.
2. **Plan**: Define scope, impacted interfaces, algorithms, and test vectors.
3. **Implement**: Keep changes minimal, coherent, and well-typed.
4. **Verify**: Execute static linters (`ament_flake8`, `ament_pep257`), unit tests (`pytest`), and simulation runs (`colcon test`).
5. **Document**: Update relevant guides, schemas, or benchmark reports.

## 3. Research Discipline

- Never convert an unverified assumption into a project claim.
- Distinguish literature facts, implementation assumptions, measured results, and research hypotheses.
- Avoid unqualified claims like "guaranteed collision-free" or "real-time" unless backed by formal proofs or empirical data under specified bounds.

## 4. ROS 2 Architectural Rules

- Target **ROS 2 Jazzy**.
- Use explicit namespaces for multi-robot isolation (`/amr_0`, `/amr_1`, etc.).
- Avoid undeclared global topics.
- Maintain consistent, valid TF trees with non-overlapping frames.
- Use parameters (`yaml` configs) rather than hardcoding robot-specific values.
- Keep message and service definitions versioned and stable.
- Keep safety-critical paths decoupled from high-level optimization modules.

## 5. Simulation & MAPF Rules

- Support Gazebo Harmonic simulation.
- Ensure world configurations and random seeds are deterministic and reproducible.
- Higher-level planners may request path adjustments but must never override low-level safety stops or hardware emergency breaks.
- All benchmark runs must record git commit hash, seed, robot count, network profile, and duration.
