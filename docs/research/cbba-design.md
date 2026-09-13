# Consensus-Based Bundle Algorithm (CBBA) Design & Formal Specification

**Author**: Lead Autonomous Engineering Agent  
**Date**: September 13, 2026  
**Status**: APPROVED DESIGN SPECIFICATION  
**Scope**: Milestone M4 — Decentralized Multi-Robot Task Allocation  

---

## 1. Theoretical Foundation

The **Consensus-Based Bundle Algorithm (CBBA)** (Choi, Brunet, How, IEEE Transactions on Robotics, Vol. 25, No. 4, August 2009) is a decentralized market-based task allocation protocol designed for multi-agent systems operating under peer-to-peer or broadcast communication.

CBBA combines:
1. **Phase 1 (Bundle Construction)**: Greedy, auction-style local bundle construction where each agent selects tasks that maximize its local marginal utility.
2. **Phase 2 (Consensus & Conflict Resolution)**: Asynchronous or synchronous message passing wherein agents exchange winning bid beliefs, resolve conflicts via a deterministic decision matrix, and cascade-drop outbid tasks and all subsequent commitments.

---

## 2. Mathematical Formulation

### 2.1. Agent Local Beliefs
Each agent $i \in \{1, \dots, N\}$ maintains:
- **Task Bundle** $b_i = (b_{i,1}, b_{i,2}, \dots, b_{i,L_i})$: An ordered sequence of assigned tasks, ordered strictly by time of addition ($L_i \le L_t$, where $L_t$ is `max_bundle_size`).
- **Path Waypoints** $p_i$: The sequence of 2D coordinates (pickups and dropoffs) visited to execute $b_i$.
- **Winning Bids Vector** $y_i \in \mathbb{R}^M$: Local belief of the highest bid placed across the fleet for each task $j \in \{1, \dots, M\}$.
- **Winning Robots Vector** $z_i \in \{1, \dots, N\}^M$: Local belief of which robot currently holds the winning bid for task $j$.
- **Timestamps Vector** $s_i \in \mathbb{R}^M$: The time epoch when belief $(y_{ij}, z_{ij})$ was recorded or updated.

### 2.2. Marginal Utility Scoring & Diminishing Marginal Utility (DMU)
Let candidate task $j$ have base priority $P_j \in \{1, 2, 3, 4\}$, pickup pose $\mathbf{p}_j^{\text{pick}}$, dropoff pose $\mathbf{p}_j^{\text{drop}}$, and deadline $D_j$.
When agent $i$ evaluates inserting task $j$ at position $m$ into candidate route $p_i \oplus_m \{j\}$:

$$
d_i(j \mid p_i) = \text{cumulative Euclidean transit distance from agent's current position to completing task } j
$$

The expected completion time is:

$$
t_i^j = t_{\text{current}} + \frac{d_i(j \mid p_i)}{v_{\text{nominal}}}
$$

The net marginal utility $U_i(j \mid p_i)$ is formulated as:

$$
U_i(j \mid p_i) = w_{\text{prio}} \cdot P_j - w_{\text{dist}} \cdot d_i(j \mid p_i) - w_{\text{late}} \cdot \max(0, t_i^j - D_j)
$$

Discounted marginal reward:

$$
c_{ij}(p_i) = \lambda^{|b_i|} \cdot \max(\epsilon_{\text{min}}, U_i(j \mid p_i))
$$

where:
- $w_{\text{prio}} = 100.0$: Emphasizes high-priority logistical deliveries.
- $w_{\text{dist}} = 10.0$: Penalizes physical distance and transit energy.
- $w_{\text{late}} = 5.0$: Imposes linear penalty for deadline exceedance.
- $\lambda = 0.95$: Diminishing factor for bundle capacity progression.
- $\epsilon_{\text{min}} = 0.01$: Guarantees positive bid feasibility.

#### Theorem (Diminishing Marginal Utility)
Because $d_i(j \mid p_i)$ represents the cumulative route distance from the robot's physical starting position, any preceding task added before $j$ strictly increases $d_i(j \mid p_i)$, monotonically reducing $U_i(j \mid p_i)$ and strictly bounding $c_{ij}$. This satisfies the DMU condition ($c_{ij}(p \oplus \{k\}) \le c_{ij}(p)$), guaranteeing convergence and eliminating bidding oscillations.

---

## 3. Consensus Decision Matrix (Table 1)

When agent $i$ receives bid broadcast $(y_k, z_k, s_k)$ from peer agent $k$ regarding task $j$:

| Peer Winner $z_{kj}$ | Self Winner $z_{ij}$ | Action Rule / Condition | Resulting Belief $(z_{ij}, y_{ij}, s_{ij})$ |
| :--- | :--- | :--- | :--- |
| **$k$** (Sender claims) | **$i$** (Self claims) | `_is_bid_higher(y_k, k, y_i, i)` | If peer higher: $(k, y_{kj}, s_{kj})$, else leave $(i, y_{ij}, s_{ij})$ |
| **$k$** | **$k$** | Peer updates bid | $(k, y_{kj}, s_{kj})$ |
| **$k$** | **$\emptyset$** (Unassigned)| Peer takes task | $(k, y_{kj}, s_{kj})$ |
| **$k$** | **$m$** (Third party) | `s_kj > s_ij` or `_is_bid_higher(y_k, k, y_i, m)` | If true: $(k, y_{kj}, s_{kj})$, else leave |
| **$i$** (Peer says self) | **$i$** | Both agree self is winner | Leave |
| **$i$** | **$k$** | Peer yielded to self | Reset: $(\emptyset, 0.0, s_{kj})$ |
| **$i$** | **$m$** | Outdated or relayed | Leave |
| **$\emptyset$** (Peer released)| **$k$** | Peer released task | Reset: $(\emptyset, 0.0, s_{kj})$ |
| **$\emptyset$** | **$m$** | Relayed release ($s_{kj} > s_{ij}$) | Reset: $(\emptyset, 0.0, s_{kj})$ |
| **$\emptyset$** | **$i$** | Self knows it holds bid | Leave |
| **$m$** (Peer says 3rd) | **$i$** | `_is_bid_higher(y_k, m, y_i, i)` | If $m$ beats self: $(m, y_{kj}, s_{kj})$, else leave |
| **$m$** | **$k$** | Peer updated winner to $m$ | $(m, y_{kj}, s_{kj})$ |
| **$m$** | **$\emptyset$** | Previously unassigned | $(m, y_{kj}, s_{kj})$ |
| **$m$** | **$m$** | Both agree $m$ won ($s_{kj} \ge s_{ij}$) | Update: $(m, y_{kj}, s_{kj})$ |
| **$m$** | **$m'$** (Diff 3rd) | `s_kj > s_ij` or `_is_bid_higher(y_k, m, y_i, m')` | If true: $(m, y_{kj}, s_{kj})$, else leave |

### 3.1. Cascade Drop Rule
If agent $i$ loses task $b_{i,n}$ (index $n$ in its bundle $b_i$), all subsequent commitments $b_{i,n+1}, \dots, b_{i,|b_i|}$ were bid upon conditionally and are no longer optimal. Agent $i$ must:
1. Truncate bundle: $b_i \leftarrow (b_{i,1}, \dots, b_{i,n-1})$.
2. Release dropped tasks: for all $d \in \{b_{i,n}, \dots, b_{i,|b_i|}\}$, if $z_{id} == i$:
   $$z_{id} \leftarrow \emptyset, \quad y_{id} \leftarrow 0.0, \quad s_{id} \leftarrow t_{\text{current}}$$
Setting $s_{id} \leftarrow t_{\text{current}}$ ensures the release is broadcast with a newer timestamp, eliminating ghost bids and preventing cycles.

### 3.2. Deterministic Tie-Breaking
For identical bids within floating-point tolerance ($\epsilon = 10^{-6}$):
$$
\text{winner}(A, B) = \begin{cases} A & \text{if } \text{robot\_id}_A < \text{robot\_id}_B \\ B & \text{otherwise} \end{cases}
$$
This ensures 100% bitwise determinism and prevents deadlock during symmetric fleet evaluations.

---

## 4. ROS 2 Communication Architecture

```
                                  /tasks/available
                                          │
                     ┌────────────────────┼────────────────────┐
                     ▼                    ▼                    ▼
             +---------------+    +---------------+    +---------------+
             | amr_0 CBBA    |    | amr_1 CBBA    |    | amr_N CBBA    |
             | Node          |    | Node          |    | Node          |
             +---------------+    +---------------+    +---------------+
               ▲     │              ▲     │              ▲     │
  /amr_0/odom  │     │              │     │              │     │  /amr_N/odom
               │     └───────┐      │     └───────┐      │     └───┐
               │             ▼      │             ▼      │         ▼
               │       ===================================     │
               └───────║        /fleet/cbba_bids         ║─────┘
                       ║    (Peer-to-Peer Broadcast)     ║
                       ===================================
                                      │
                                      ▼
                        /tasks/update_status (M3 Lifecycle)
                        /amr_*/bundle        (Dashboard / M5)
```

1. **`/fleet/cbba_bids` (`amr_fleet_msgs/msg/CBBABid`)**: Broadcast channel where each robot agent shares its current bid and winner vector.
2. **`/${robot_id}/odom` (`nav_msgs/msg/Odometry`)**: Robot localized position providing dynamic coordinates for distance calculations.
3. **`/tasks/available` (`amr_fleet_msgs/msg/TaskList`)**: Available tasks published by the M3 Task Manager.
4. **`/${robot_id}/bundle` (`amr_fleet_msgs/msg/RobotBundle`)**: Allocated tasks and convergence status published for observability and downstream planning.
5. **`/tasks/update_status` (`amr_fleet_msgs/msg/TaskEvent`)**: Signals task state transition (`PENDING` $\rightarrow$ `ASSIGNED`) upon stable consensus convergence.
