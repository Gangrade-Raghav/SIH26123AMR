# Milestone M7 Research: Fleet Resilience Protocol

## 1. Resilience Philosophy & Safety Invariant

In safety-critical AMR fleet deployments, communication networks cannot be assumed to be 100% reliable. The fundamental resilience philosophy of the NRDAS architecture is:

$$\text{Local Safety} \perp \text{Inter-Robot Communication}$$

- **Physical Collision Invariant**: No communication failure (whether latency, packet loss, burst blackout, or network partitioning) can ever cause two AMRs to physically collide. Local reactive safety braking (via 360-degree LiDAR) acts independently and unconditionally at 10 Hz at the motor boundary.
- **Coordination Degradation**: Communication degradation impacts *system efficiency* (makespan, consensus latency, replanning frequency), but NEVER *physical safety*.

---

## 2. Information Freshness Lifecycle

Information received from remote peers undergoes age tracking relative to local wall/simulation time:
$$\Delta t = t_{\text{current}} - t_{\text{last\_received}}$$

```
  +----------------+      Age >= 1.5s       +----------------+      Age >= 4.0s       +----------------+
  |    CURRENT     +----------------------->+     STALE      +----------------------->+    EXPIRED     |
  | (Full Trust)   |                        | (Caution Mode) |                        | (Prune Ledger) |
  +-------+--------+                        +-------+--------+                        +-------+--------+
          ^                                         ^                                         |
          |               New Packet Arrives        |                                         |
          +-----------------------------------------+-----------------------------------------+
                                                     Reconnection Declared
```

### 2.1 State Definitions
- **`CURRENT` ($\Delta t < 1.5\text{ s}$)**: Remote reservations and CBBA winning bids are actively fresh. Standard PIBT prioritization and space-time reservation filtering apply.
- **`STALE` ($1.5\text{ s} \le \Delta t < 4.0\text{ s}$)**: Peer has missed consecutive heartbeats. The local planner flags a staleness warning, ceases aggressive velocity progression towards contested intersections, and adopts a conservative wait posture if heading towards that peer's last claimed cells.
- **`EXPIRED` ($\Delta t \ge 4.0\text{ s}$)**: Peer is declared silent/disconnected.
  - **Deadlock Phantom Prevention**: If remote reservations belonging to a disconnected peer were never purged, they would permanently lock cells in the space-time reservation table, freezing other robots indefinitely.
  - **Pruning**: `StaleStateManager.prune_expired_reservations(table)` systematically releases all space-time reservations registered by the expired peer.
  - **Local Path Fallback**: The local robot can replan through the newly freed corridor while relying on local LiDAR safety in case the silent robot is physically stalled in that aisle.

---

## 3. Reconnection and State Reconciliation

When a silent or partitioned robot re-establishes connectivity (first message received after being in `EXPIRED` or `MISSING` state):

1. **Reconnection Event**: The receiving peer's `StaleStateManager` flags `is_reconnection = True` and increments `reconnection_events_count`.
2. **State Reconciliation Re-Broadcast**:
   - The AMR immediately publishes its current bundle and bids to `/fleet/cbba_bids`.
   - The AMR re-publishes its active space-time reservation trajectory window $[t, t + h]$ to `/fleet/reservations`.
   - The AMR broadcasts its current coordination status to `/fleet/coordination_status`.
3. **Ledger Resynchronization**: Peers update their local reservation tables with the freshly arrived reservations, resolving any divergent space-time claims deterministically using the M6 priority rules:
   $$\text{Priority} = 1000 \cdot \text{TaskPriority} + \text{RemainingDistanceCost} + \text{RobotIDTieBreaker}$$

---

## 4. Behavior Under Sub-Fleet Network Partitioning

When the fleet is partitioned into two disjoint groups $A = \{r_0, r_1, r_2\}$ and $B = \{r_3, r_4\}$:
1. **Intra-Partition Consensus**: Group $A$ robots communicate freely, reaching rapid CBBA consensus and coordinating reservations without conflict within their spatial zones.
2. **Cross-Partition Isolation**: Group $A$ receives zero coordination packets from Group $B$. After 4.0 seconds, each group marks the other as `EXPIRED` and prunes remote reservations.
3. **Spatial Convergence Guarantee**: If robots from different partitions encounter each other physically (e.g. crossing the central corridor):
   - Because they lack inter-fleet reservations, both robots attempt to enter the corridor.
   - As they approach $d < 0.60\text{ m}$, local LiDAR detection flags proximity warnings.
   - At $d \le 0.35\text{ m}$, local emergency safety braking halts forward velocity to $0.0\text{ m/s}$, preventing collision.
   - Once halted, local reactive maneuvering or human/supervisor intervention restores safe passage.
