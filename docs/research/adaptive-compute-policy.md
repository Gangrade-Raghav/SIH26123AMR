# Adaptive Compute Policy Specification

**Document**: `docs/research/adaptive-compute-policy.md`  
**Milestone**: M8B  
**Component**: `amr_fleet_core.adaptive_compute_policy.AdaptiveComputePolicy`  

---

## 1. Policy Configuration & Design Thresholds

The policy operates according to the selected operating parameters specified in `AdaptiveComputePolicyConfig`:

```python
@dataclass(frozen=True)
class AdaptiveComputePolicyConfig:
    # Anti-oscillation
    min_dwell_time_sec: float = 3.0
    confirmation_samples: int = 3

    # Priority 1: Network / Staleness thresholds
    stale_age_low_threshold_s: float = 1.5
    stale_age_normal_threshold_s: float = 0.8
    comm_loss_low_threshold: float = 0.15
    comm_loss_normal_threshold: float = 0.05

    # Priority 2: Host Compute thresholds
    cpu_low_threshold_pct: float = 85.0
    cpu_normal_threshold_pct: float = 70.0
    planning_budget_factor: float = 1.2

    # Priority 3: Spatial Contention thresholds
    conflict_high_threshold: int = 2
    conflict_normal_threshold: int = 0

    # Fail-safe
    stale_state_timeout_sec: float = 5.0
```

---

## 2. Formal Decision Logic

The policy evaluates incoming telemetry against four strict priority tiers:

### Priority Tier 1: Communication Impairment & State Age
- **Condition for LOW**:
  $$\bar{\Delta}_{\text{peer}} \ge 1.5\,\text{s} \quad \lor \quad \hat{p}_{\text{loss}} \ge 0.15 \quad \lor \quad \text{outage} = \text{True}$$
- **Rationale**: When peer information is stale or message loss is severe, high-frequency replanning ($2.0\,\text{Hz}$ or $4.0\,\text{Hz}$) causes thrashing against phantom reservations and wastes scarce channel bandwidth. Downscaling to $1.0\,\text{Hz}$ with horizon $h=6$ stabilizes local path following and conserves wireless transmission.

### Priority Tier 2: Host CPU Pressure
- **Condition for LOW**:
  $$u_{\text{cpu}} \ge 85.0\% \quad \lor \quad \bar{\tau}_{\text{plan}} > 1.2 \cdot \tau_{\text{budget}}$$
- **Rationale**: If the host CPU is saturated or planning cycles exceed their time budget, continuing to replan at high rates degrades OS scheduling and increases latency jitter. Reducing replan rate to $1.0\,\text{Hz}$ sheds computational load.

### Priority Tier 3: Spatial Traffic Contention
- **Condition for HIGH**:
  $$c_{\text{active}} \ge 2 \quad \lor \quad d_{\text{yield}} = \text{True}$$
- **Rationale**: In congested bottleneck regions (corridors, intersections) where peer trajectories overlap, planning further ahead ($h=14$) with smaller execution steps ($w=2$) and double the replan frequency ($4.0\,\text{Hz}$) allows agents to resolve mutual conflicts rapidly before reaching minimum separation limits.

### Priority Tier 4: Nominal Recovery
- **Condition for NORMAL**:
  $$\bar{\Delta}_{\text{peer}} \le 0.8\,\text{s} \quad \land \quad \hat{p}_{\text{loss}} \le 0.05 \quad \land \quad u_{\text{cpu}} \le 70.0\% \quad \land \quad c_{\text{active}} == 0$$
- **Rationale**: When network conditions are healthy and traffic contention clears, robots transition smoothly back to balanced baseline coordination ($2.0\,\text{Hz}, h=10, w=4$).

---

## 3. Fail-Safe Fallback Guarantees

1. **State Age Expiry**: If no telemetry state has been received for $> 5.0\,\text{s}$ (`stale_state_timeout_sec`), the policy automatically forces an immediate fallback to `COMPUTE_MODE_NORMAL`.
2. **Exception Safety**: Any unhandled exception during policy evaluation triggers an internal `try...except` block that resets the active mode to `COMPUTE_MODE_NORMAL` and logs an emergency audit trace:
   ```python
   except Exception as exc:
       return self.force_fallback(f"UNHANDLED_EXCEPTION: {exc}")
   ```
3. **Invalid Mode Recovery**: If an invalid string or unsupported mode is encountered, `get_compute_mode_config()` defaults deterministically to `COMPUTE_MODE_NORMAL`.

---

## 4. Mode Transition Event Format

Every mode change emits a `ComputeModeEvent` message defined in `amr_fleet_msgs/msg/ComputeModeEvent.msg`:

```
std_msgs/Header header
string robot_id
string previous_mode
string current_mode
string trigger_signal
float64 trigger_value
float64 threshold_value
string reason
float64 dwell_time_sec
```

This enables complete post-mission provenance auditing and verifiable traceability.
