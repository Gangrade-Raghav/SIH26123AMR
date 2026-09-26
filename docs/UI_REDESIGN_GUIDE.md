# NRDAS Industrial Fleet Operations Console — UI/UX Operator Guide (V2)

## 1. Overview & Design Rationale

The **NRDAS Fleet Operations Console (V2)** is an industrial-grade, mission-critical interface designed specifically for autonomous mobile robot (AMR) warehouse orchestration. Built on the core principle of **Visual Clarity > Information Density**, the console avoids the visual clutter of generic admin dashboards, muddy palettes, and decorative telemetry.

The interface serves as the primary operational surface for monitoring decentralized fleet coordination, inspecting real-time CBBA (Consensus-Based Bundle Algorithm) auctions, and dispatching/controlling warehouse tasks.

---

## 2. Dual Color System & Light / Dark Mode Toggle

The console provides two purpose-built industrial color schemes accessible via the **`☀️ LIGHT / 🌙 DARK`** toggle button in the top command header:

### 2.1 Light Mode (White + Industrial Orange + Grey Accents)
- **Outer Canvas Background**: `#f8fafc` (clean, soft off-white).
- **Surface Panels & Cards**: `#ffffff` (pure crisp white).
- **Sub-Panels, Form Inputs & Table Headers**: `#f1f5f9` (light slate container surface).
- **Subtle & Medium Borders**: `#e2e8f0` (subtle boundaries) and `#cbd5e1` (distinct separators).
- **Typography**: `#0f172a` (high-contrast charcoal black for headers and primary numbers), `#475569` (slate grey for metadata), and `#64748b` (muted grey for timestamps and units).
- **Primary Brand Accent**: `#FF7A00` Industrial Orange for action buttons (`+ CREATE TASK`), active selections, and route corridors.
- **Warehouse Map Floor**: Architectural off-white (`#f8fafc`) with `#e2e8f0` (1m) and `#cbd5e1` (5m) grid lines, dark slate storage racks (`#334155` with `#1e293b` borders and `#475569` cross-hatch), and `#64748b` aisle labels.

### 2.2 Dark Mode (Obsidian Black + Industrial Orange + White/Grey Accents)
- **Outer Canvas Background**: `#000000` (pure pitch black / true obsidian black).
- **Surface Panels & Cards**: `#0a0a0a` (pure obsidian black container surfaces).
- **Sub-Panels, Form Inputs & Table Headers**: `#121212` (elevated dark obsidian surface).
- **Subtle & Medium Borders**: `#222222` (subtle neutral dark boundaries) and `#333333` (distinct neutral grey separators).
- **Typography**: `#ffffff` (crisp pure white for headers, icons, and primary numbers), `#a1a1aa` (neutral cool grey for metadata), and `#71717a` (muted grey for timestamps and units).
- **Primary Brand Accent**: `#FF7A00` Industrial Orange.
- **Warehouse Map Floor**: Pure obsidian black floor (`#000000`) with `#141414` (1m) and `#262626` (5m) neutral grid lines, `#141414` deep black storage racks with `#2e2e2e` borders and `#383838` cross-hatch, and `#71717a` aisle labels.

### 2.3 Mode Toggle & Persistence
- **Header Control**: The toggle button is located in the top-right system metrics cluster:
  - `☀️ LIGHT`: When in dark mode, click to activate light mode.
  - `🌙 DARK`: When in light mode, click to activate dark mode.
- **Instant Transformation**: Toggling updates `data-theme` on the root document element and instantly triggers `initWarehouseStaticLayers()`, converting both HTML cards and SVG map canvas elements synchronously without reloading.
- **Persistent Memory**: Operator preference is saved to `localStorage.getItem('nrdas_theme')`, automatically preserving the chosen theme across page refreshes.

### 2.4 Semantic Status Mapping (Consistent Across Both Themes)
- **Status Green** (`#10b981` dark / `#15803d` light): Online state, active moving/picking, 0 collisions, completed tasks.
- **Status Blue** (`#3b82f6` dark / `#0369a1` light): Charging state, assigned/bidding tasks, alternative deconfliction routes.
- **Status Amber** (`#f59e0b` dark / `#b45309` light): Warnings, pending tasks, requeued states, charging station glyphs.
- **Status Red** (`#ef4444` dark / `#b91c1c` light): Emergency stops, collisions, failed tasks.

---

## 3. Four-Quadrant Layout Hierarchy

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                   TOP HEADER BAR                                       │
│ NRDAS ///   │ FLEET: 10/10 ONLINE │ MOVING: 2 │ CHARGING: 3 │ CONFLICTS: 0 │ SIM: 145.7s│
├─────────────────────┬───────────────────────────────────────────┬──────────────────────┤
│                     │                                           │                      │
│ COMMUNICATION LOG   │          WAREHOUSE MAP (HERO)             │ ROBOT STATUS         │
│ (21% Width)         │          (58% Width)                      │ (21% Width)          │
│                     │                                           │                      │
│ - Category filters  │ - Uncluttered warehouse geometry          │ - Compact AMR cards  │
│ - Compact row feed  │ - Small directional pucks (no giant cards)│ - State, battery,    │
│ - [TASK], [CBBA],   │ - Single contextual selection popup       │   task route, prog % │
│   [PLAN], [SAFETY]  │ - Subordinate route visualization         │ - Bottom state tally │
│                     │ - Compact corner legend & zoom controls   │                      │
├─────────────────────┴───────────────────────────────────────────┴──────────────────────┤
│                       TASK OPERATIONS (SECONDARY HERO)                                 │
│ AVAILABLE | ASSIGNED | IN PROGRESS | COMPLETED | ALL                       [+ CREATE TASK]│
│ Table: ID | Route | Priority | Robot | State | Progress | Created | Actions           │
└────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Component Deep-Dive

### 4.1 Top Header Bar
- **Brand**: `NRDAS ///` with subtitle `NETWORK RESILIENT • DECENTRALIZED • AUTONOMOUS • SYSTEMS`.
- **Fleet Metric Cluster**: Single cohesive row with subtle vertical divider lines (`1px solid #1f293d`):
  - `FLEET`: `10 / 10 ONLINE` (bold emerald)
  - `MOVING`: Dynamic count of robots executing trajectory windows
  - `CHARGING`: Count of docked or charging AMRs
  - `IDLE`: Available robots awaiting tasks
  - `CONFLICTS`: Dynamic count of space-time reservation deconflictions
  - `COLLISIONS`: Cumulative physical contacts / E-stops (zero in nominal decentralized operation)
- **System Cluster**:
  - `SYSTEM`: `● ONLINE` (pulsing emerald beacon)
  - `COMM`: Active network profile (`NORMAL`, `LOSS_LOW`, `LOSS_HIGH`)
  - `COMPUTE`: Active planning compute mode (`ADAPTIVE` / `FIXED`)
  - `SIM TIME`: Ground-truth simulation clock in seconds (`145.7 s`).
- **Eliminated**: All decorative wall clocks, fake percentages, and oversized rectangular cards.

### 4.2 Left Panel: Communication Log (21% Width)
- **Header**: `COMMUNICATION` with pulsing live indicator.
- **Filter Bar**: Fast filtering across coordination domains:
  - `ALL`: Complete fleet message stream.
  - `CBBA`: Decentralized auction bid broadcasts and bundle convergence messages.
  - `TASK`: Task lifecycle events (`CREATED`, `ASSIGNED`, `IN_PROGRESS`, `COMPLETED`, `CANCELLED`).
  - `PLAN`: Space-time deconflictions, velocity modulation, and rerouting notifications.
  - `SAFETY`: Wait-For-Graph (WFG) deadlock cycle recovery actions and safety stops.
  - `SYSTEM`: Network profile shifts, node discovery, and middleware transport health.
- **Compact Row Format**:
  - Timestamp (`14:32:07` monospace) | Sender $\to$ Recipient (`AMR-02 → AMR-03`) | Domain Tag (`[TASK]`, `[CBBA]`, etc.).
  - Readable event summary text with subtle left accent on the latest event.

### 4.3 Center Panel: Warehouse Map Live Fleet View (58% Width — Hero Element)
- **Dominant Visual Center**: The warehouse map occupies the majority of the screen space on an architectural dark slate floor (`#0a101d`) with subtle 1m and 5m gridlines.
- **Facility Infrastructure**:
  - **Storage Racks**: High-contrast dark navy blocks (`#182336`) with subtle outline and cross-hatch interior.
  - **Aisle Markers**: Technical, low-visual-weight aisle labels (`A1`, `A2`, `B1`, `B2`, `C1`, `C2` in `#475569`).
  - **Operational Zones**: Bounded by light dashed outlines (`PICKUP ZONE` green, `DROPOFF ZONE` blue, `CHARGING STATION ⚡` amber).
- **Robot Map Markers**:
  - Small directional circular puck (radius 8.5px) with heading needle pointing along robot yaw $\theta$.
  - State ring: Green for moving/picking, blue for charging, slate for idle.
  - Compact text tag below: `● AMR-03` (9px).
  - **Zero permanent clutter**: No persistent giant info boxes obscuring the aisles.
- **Contextual Single Selection Popup**:
  - Clicking any robot displays **exactly one** contextual callout anchored to that robot:
    ```
    ┌───────────────────────────┐
    │ AMR-03           MOVING   │
    │ Battery: 62%              │
    │ Task: T043                │
    │ Progress: 35%             │
    │ Position: (14.2, 8.5)     │
    └───────────────────────────┘
    ```
  - Dismissible via `✕` or clicking outside. All other robots remain unobtrusive pucks.
- **Route Visualization**:
  - **Current Active Path**: Solid orange line (`#FF7A00`, 2px stroke, `opacity: 0.85`), with small waypoint beads.
  - **Planned Path**: Thin dashed neutral line (`#64748b`, 1.5px stroke, `opacity: 0.7`).
  - **Alternative Path**: Thin dotted blue line (`#3b82f6`, 1.5px stroke, `opacity: 0.6`).
  - When an AMR is selected: Its trajectory is brought to full opacity (`1.0`), while other robots' paths are dimmed to `0.15` opacity.
- **Map Legend & Controls**:
  - Compact legend in bottom-left corner (`PATH: ■ CURRENT ┄ PLANNED ⋯ ALTERNATE | MAP: □ RACK □ ZONE ● ROBOT`).
  - Zoom (`−`, `100%`, `+`) and projection toggle (`2D Flat` vs `3D Isometric`) in top-right corner.

### 4.4 Right Panel: Robot Status (21% Width)
- **Header**: `ROBOT STATUS` with live online count.
- **Compact Robot Cards**:
  - Header: AMR ID (`AMR-03`) with status pill (`● MOVING`, `● CHARGING`, `● IDLE`).
  - Battery Gauge: Percentage with slim 4px progress bar (turns red when $<25\%$).
  - Speed & Route: Current linear speed ($v$ in m/s) and active pickup/dropoff coordinates.
  - Subgoal Progress: Visual progress bar towards next waymark or delivery point.
  - Interactive: Clicking any card selects the robot on the map and focuses its trajectory.
- **Bottom State Tally**: Real-time distribution across operational states (`MOV 2 | PICK 1 | CHG 3 | IDLE 4 | ERR 0`).

### 4.5 Bottom Panel: Task Operations (Secondary Hero — Full Width)
- **Header**: `TASK OPERATIONS` with tab filters:
  - `ALL`: Complete task catalog.
  - `AVAILABLE`: Unassigned tasks awaiting CBBA bidding.
  - `ASSIGNED`: Tasks won by an AMR and queued in its bundle.
  - `IN PROGRESS`: Tasks currently under active execution.
  - `COMPLETED`: Fulfilled missions.
- **Action Button**: `+ CREATE TASK` (styled in `#FF7A00`).
- **Clean Table Layout**:
  - Columns: `ID`, `ROUTE (PICKUP → DROPOFF)`, `PRIORITY`, `ASSIGNED AMR`, `STATUS`, `PROGRESS`, `CREATED`, `ACTIONS`.
  - Status Badges: Distinct, restrained color-coding (`PENDING` amber, `BIDDING` blue, `ASSIGNED` sky blue, `IN_PROGRESS` green, `COMPLETED` emerald, `CANCELLED` gray, `REQUEUED` yellow).
  - Actions:
    - `[ VIEW ]`: Opens the Task Detail slide-over drawer.
    - `[ CANCEL ]`: Cancels an active or pending task via ROS 2 `/tasks/control`.
    - `[ REQUEUE ]`: Re-injects a cancelled or failed task into the pending pool.

### 4.6 Task Detail Slide-Over Drawer
- Slides smoothly from the right edge of the viewport upon clicking a task row or `VIEW`:
  - **Task Header**: Task ID and status badge.
  - **Route Specification**: Formatted pickup and dropoff coordinate pairs.
  - **Assignment & Consensus**: Assigned robot ID, CBBA convergence state (`● CONVERGED`), and execution progress.
  - **Integrated CBBA Allocation Inspector**:
    - Winning robot and winning marginal cost value.
    - Ranked bids table listing all participating AMRs, bid values, and winner tags.
  - **Action Controls**: In-drawer `[ CANCEL TASK ]` and `[ REQUEUE TASK ]`.

### 4.7 Create Task Modal Dialog
- In-page modal with backdrop blur:
  - **Pickup & Dropoff Coordinates**: Numeric $X, Y$ inputs with quick preset chips (`Zone A → B`, `Zone B → C`, `Zone C → D`).
  - **Priority**: Dropdown (`LOW (1)`, `NORMAL (2)`, `HIGH (3)`, `CRITICAL (4)`).
  - **Assignment Method**:
    - `AUTO (Decentralized CBBA Auction)`: Broadcast to fleet for competitive marginal bidding.
    - `DIRECT (Dedicated Robot Override)`: Constrains auction bidding exclusively to the chosen AMR.
  - **Requested Robot**: Auto-populated dropdown of active AMRs (enabled only in DIRECT mode).
  - **Deadline**: Optional simulation deadline in seconds.
  - Submits asynchronously via POST `/api/task/create` to ROS 2 service `/tasks/create`.

---

## 5. Telemetry & Data Integrity Principles

The console operates strictly on ground-truth telemetry:
1. **Zero Mock Telemetry**: All numbers, states, positions, and logs stem from real ROS 2 topics and graph state.
2. **Fallback Transparency**: If a topic or metric is not available in the current simulation mode, it is displayed as `—` or `N/A` rather than fabricating numbers.
3. **No Decorative KPIs**: Arbitrary efficiency percentages, fake wall clocks, and vanity graphs are strictly prohibited.
