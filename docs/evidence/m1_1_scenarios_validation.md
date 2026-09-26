# NRDAS-FR Milestone 1.1 Scenarios Validation Evidence

**Execution Timestamp**: 2026-09-23T22:27:47.188656
**Overall Outcome**: **100% PASSED**
**Scenarios Evaluated**: 6 / 6

---

## Executive Summary Matrix
| Scenario ID | Scenario Description | Status | Key Invariant Verified |
| :--- | :--- | :---: | :--- |
| **M1-A** | Single Robot Hard Kill during Task Execution | `PASSED` | Zero task duplication, task reclaimed & reallocated |
| **M1-B** | Heartbeat Timeout in Narrow Corridor | `PASSED` | Debounced timeout detection & aisle rerouting |
| **M1-C** | Simultaneous Dual Detection Race | `PASSED` | Idempotent CAS prevents split-brain duplicate ownership |
| **M1-D** | Actuator Failure & Chassis Avoidance | `PASSED` | Clearance 1.0m maintained (> 0.45m threshold) |
| **M1-E** | Operator Restoration & Re-integration | `PASSED` | Chassis obstacle cleared, robot re-enters CBBA auction |
| **M1-F** | Comm Loss vs Node Kill Distinction | `PASSED` | Comm dropout detected as COMM_LOSS without spurious reclaim |

---

## Provenance & Ground Truth Audit
> **Contact Sensor Provenance**: Contact detection in benchmarks is verified using a **2D Oriented Bounding Box (OBB) Geometric Proxy** executing the Separating Axis Theorem (SAT) on live odometry poses with an envelope buffer of $0.45\text{m}$. No raw physics bumper contact sensor is present in the hardware model. All collision freedom claims are strictly bounded by this geometric proxy.
