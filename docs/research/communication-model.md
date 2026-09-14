# Milestone M7 Research: Communication Impairment Abstraction

## 1. Motivation and Problem Formulation

In industrial multi-robot warehouse installations, wireless communication over Wi-Fi, 5G, or mesh radios experiences non-ideal physical phenomena:
1. **Multipath fading and structural RF occlusion**: Metal shelving racks, concrete pillars, and high-density inventory induce severe attenuation and burst dropouts.
2. **Bandwidth contention and AP handoffs**: Roaming between access points causes transient latency spikes (100–500 ms) and jitter.
3. **Partial network partitioning**: Deep warehouse zones or RF shadow regions isolate sub-fleets from each other.

To evaluate fleet coordination resilience scientifically, the communication impairment model must be:
- **Reproducible**: Governed by deterministic pseudorandom seeds ($S$).
- **Configurable**: Parameterized across latency, jitter, loss rates, burst lengths, and outage intervals.
- **Architecturally Pure**: Injected solely at the ROS 2 transport boundary without modifying core path planning or collision avoidance algorithms.

---

## 2. Mathematical Modeling of Impairment Modes

### 2.1 Transport Latency and Jitter
The delivery timestamp $t_{\text{deliver}}$ for a message sent at time $t_{\text{sent}}$ is given by:
$$t_{\text{deliver}} = t_{\text{sent}} + \frac{\tau_{\text{eff}}}{1000}$$
where:
$$\tau_{\text{eff}} = \max\left(0.0, \tau_{\text{base}} + \mathcal{U}(-\Delta_j, +\Delta_j)\right)$$
- $\tau_{\text{base}} \ge 0$: Configured baseline latency (ms).
- $\Delta_j \ge 0$: Symmetrical jitter amplitude (ms).
- $\mathcal{U}(-\Delta_j, +\Delta_j)$: Uniform random distribution.

### 2.2 Independent Random Packet Loss
Each coordination message undergoes a Bernoulli trial with drop probability $p_{\text{loss}} \in [0.0, 1.0]$:
$$\text{Action} = \begin{cases} \text{DROP}, & \text{if } \xi \sim \mathcal{U}(0, 1) < p_{\text{loss}} \\ \text{DELIVER}, & \text{otherwise} \end{cases}$$

### 2.3 Gilbert-Elliott Two-State Markov Burst Loss Model
Packet drops in wireless environments typically exhibit temporal correlation rather than memoryless independence. The Gilbert-Elliott model captures this through a 2-state discrete-time Markov chain:
- **State $G$ (Good)**: Clean transmission ($p_{\text{loss}|G} \approx 0$).
- **State $B$ (Bad)**: Deep fade / burst loss streak ($p_{\text{loss}|B} = 1.0$).

The transition probability matrix is:
$$P = \begin{pmatrix} 1 - p_{\text{burst\_start}} & p_{\text{burst\_start}} \\ p_{\text{recover}} & 1 - p_{\text{recover}} \end{pmatrix}$$
where:
$$p_{\text{recover}} = \frac{1}{\mu_{\text{burst\_length}}}$$
- $p_{\text{burst\_start}}$ is the probability of entering a fade.
- $\mu_{\text{burst\_length}}$ is the mean number of consecutive dropped packets before channel recovery.

```
       1 - p_burst_start                 1 - p_recover
          +---------+                       +---------+
          |         |                       |         |
          v         |                       v         |
     +---------+    |  p_burst_start   +---------+    |
     |  GOOD   +----+----------------->+   BAD   +----+
     |  STATE  |<----------------------+  STATE  |
     +---------+      p_recover        +---------+
    (Pass packet)                    (Drop packet)
```

### 2.4 Outage Windows
An outage is modeled as a deterministic or time-triggered disconnection interval:
$$\text{IsOutage}(t) = \begin{cases} \text{True}, & t_{\text{start}} \le t < t_{\text{start}} + \tau_{\text{outage}} \\ \text{False}, & \text{otherwise} \end{cases}$$
During active outage, all inter-robot coordination packets are dropped unconditionally.

### 2.5 Sub-Fleet Network Partitioning
Given an isolated robot set $\mathcal{R}_{\text{isolated}} \subset \mathcal{R}_{\text{fleet}}$:
$$\text{IsPartitioned}(r_{\text{sender}}, r_{\text{receiver}}) = (r_{\text{sender}} \in \mathcal{R}_{\text{isolated}}) \oplus (r_{\text{receiver}} \in \mathcal{R}_{\text{isolated}})$$
Messages exchanged between members of the same sub-fleet pass unimpaired; messages crossing partition boundaries are dropped with reason `PARTITION`.

---

## 3. Implementation Details (`communication_model.py`)

1. **`CommunicationProfileConfig`**: Dataclass defining the full impairment parameter vector (`profile_name`, `enabled`, `latency_ms`, `jitter_ms`, `loss_probability`, `burst_loss_probability`, `burst_length_mean`, `outage_start_s`, `outage_duration_s`, `seed`, `isolated_robots`).
2. **`CommunicationImpairmentModel`**: Core evaluator instantiated on each AMR node. Evaluates `process_message(sender_id, receiver_id, current_time)` returning `(action, delay_ms, drop_reason)`.
3. **`DelayedMessageQueue`**: A thread-safe, min-heap priority queue sorted by delivery timestamp (`deliver_at`). Polled at 10 Hz in the AMR high-frequency control loop, ensuring packets are dispatched at their designated delayed arrival times without thread blocking or OS context switching overhead.
