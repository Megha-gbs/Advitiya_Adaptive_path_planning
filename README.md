# Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads

### Smart India Hackathon (SIH) • Problem Statement 26037
**An L5-Oriented Software Research Prototype & Autonomy Intelligence Platform for Unstructured Indian Driving Conditions**

---

## 1. Executive Summary & The Big Idea

Autonomous driving research designed for Western highways and structured urban grid systems fails catastrophically when deployed on Indian roads. Indian traffic environments are characterized by:
- **Missing, faded, or absent lane markings & unpaved shoulders**
- **Hyper-heterogeneous traffic** (pedestrians, cows/stray animals, motorcycles slicing through narrow gaps, auto-rickshaws, buses, hand-pulled carts, heavy multi-axle trucks)
- **Informal hazards & roadway degradation** (potholes, informal speed breakers, construction debris, monsoon water logging)
- **Acoustic navigation cues** (honking as an informal signaling language rather than an emergency alarm)
- **Extreme operational variations** (dense dust, monsoon downpours, night glare, narrow urban gullies, and mountain ghat hairpin roads)

### The Core Vision: Beyond a Simple Path Planner
This project is **not** a basic reactive obstacle-avoidance script or isolated path planner. It is an **Indian-Road Autonomy Intelligence Platform** that:
1. Ingests heterogeneous real-world Indian and global driving datasets into a structured metadata catalog.
2. Builds an uncertainty-aware, predictive 4D world model and dynamic risk field $R(x, y, t)$.
3. Generates dynamically feasible, context-adaptive trajectories.
4. Validates every motion candidate through an **independent deterministic safety shield** capable of overriding AI logic.
5. Executes trajectory tracking through dynamics-constrained Model Predictive Control (MPC).
6. Closes the loop via simulation and **failure-driven continuous learning** (capturing failure edge cases, generating synthetic variants, and feeding them back to retrain models).

```
SENSE -> ESTIMATE -> FUSE -> PREDICT -> WORLD MODEL -> DECIDE -> PLAN -> SAFETY -> CONTROL -> SIMULATE -> TEST -> FEEDBACK
```

---

## 2. High-Level 12-Stage System Architecture

```
                 ┌──────────────────────────────────────────┐
                 │    1. ENVIRONMENT & SCENARIO ENGINE      │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │  2. SENSOR SIMULATION & DATA ACQUISITION │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │  3. SENSOR PREPROCESSING & CALIBRATION   │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │   4. PERCEPTION & ROAD UNDERSTANDING     │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │ 5. SENSOR FUSION, TRACKING & RISK ESTIM. │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │ 6. LOCALIZATION, MAPPING & WORLD MODEL   │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │  7. MOTION PREDICTION & FUTURE STATES    │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │   8. BEHAVIOR & ADAPTIVE PATH PLANNING   │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │  9. SAFETY SUPERVISOR & COLLISION AVOID. │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │     10. VEHICLE DYNAMICS & CONTROL       │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │ 11. CLOSED-LOOP SIMULATION & EXECUTION   │
                 └────────────────────┬─────────────────────┘
                                      ↓
                 ┌──────────────────────────────────────────┐
                 │ 12. VALIDATION, METRICS & FAILURE ANAL.  │
                 └────────────────────┬─────────────────────┘
                                      │
                                      ▼
═════════════════════════════════════════════════════════════════════════
         DATA & INTELLIGENCE MANAGEMENT LAYER (CROSS-CUTTING)
  Datasets • Database • Models • Scenarios • Experiments • Results • Failures
═════════════════════════════════════════════════════════════════════════
```

---

## 3. Data & Intelligence Management Layer

### Why a Database in an Autonomous Vehicle Project?
The database is **not** a raw storage dump for heavy binary LiDAR point clouds or multi-gigabyte video files. It is the **system's experimental memory and traceability engine**.

```
                RAW SENSOR STORAGE                        METADATA DATABASE
             (File / Object Storage)                    (Indexed SQL / Registry)
          ┌────────────────────────────┐              ┌───────────────────────────┐
          │ • Camera Images / Videos   │              │ • Frame ID & Dataset ID   │
          │ • LiDAR Point Clouds (PCD) │  file_path   │ • Timestamp & Calibration │
          │ • Radar Raw Streams        │ ───────────> │ • 2D/3D Object Labels     │
          │ • CARLA Rosbags / Scenarios│   reference  │ • Track IDs & Velocities  │
          │ • Simulation Telemetry     │              │ • Weather, Road & Region  │
          └────────────────────────────┘              │ • Model Version & Checkpt │
                                                      │ • Scenario & Failure Type │
                                                      └─────────────┬─────────────┘
                                                                    │
                                            Query: "Find all motorcycle cut-ins in heavy rain
                                                    where MPC brake override triggered"
                                                                    │
                                                                    ▼
                                                    [Traceable Root Cause & Regression]
```

### Traceability: The Closed-Loop Failure Learning Cycle
If the autonomous vehicle makes an incorrect decision during a simulation run, the system traces the decision backwards:
$$\text{Control Action} \longleftarrow \text{Planner Decision} \longleftarrow \text{Risk Field} \longleftarrow \text{Predicted Motion} \longleftarrow \text{Fused Track} \longleftarrow \text{Sensor Frame}$$
1. **Detect Failure:** Simulation flags a near-miss ($TTC < 0.8\text{s}$) or boundary breach.
2. **Catalog Failure:** Database logs `FailureID`, `ScenarioID`, `ModelVersion`, `SensorState`, `ModuleResponsible`.
3. **Generate Variants:** Synthetic data engine creates 50 variations of that exact failure scenario (varying traffic density, weather, motorcycle speed).
4. **Retrain / Re-tune:** Perception/Prediction/Planning modules are fine-tuned against the newly augmented dataset.
5. **Regression Verification:** Full scenario suite is re-executed to guarantee no behavioral regression before code merge.

---

## 4. Multi-Tier Dataset Strategy

| Tier | Dataset | Source / Scope | Primary Role in the Stack |
| :--- | :--- | :--- | :--- |
| **Tier A (Indian Core)** | **IDD (Indian Driving Dataset)** | 10k images, 34 classes, Hyderabad & Bangalore | Indian traffic taxonomy, 2D object detection, semantic segmentation |
| | **IDD-3D** | ~12k LiDAR frames, 223k annotated objects, 17 classes | 3D bounding box detection, camera-LiDAR fusion, occlusion handling |
| | **Roadscapes** | 9k Indian road images across urban, rural & highway | Diverse rural/urban road boundary segmentation, lighting robustness |
| | **IDS-JODHA** | 87k video frames, 16k synchronized LiDAR scans, rain/dust | Multimodal temporal tracking, monsoon weather robustness |
| | **DriveIndia / IndiaScene365** | Multimodal Indian driving sequences | Extreme unstructured road context & lane-free navigability |
| **Tier B (Global Transfer)** | **nuScenes** | 1,000 multimodal scenes (6 cam, 1 LiDAR, 5 radar) | Radar-camera-LiDAR velocity fusion, tracking baseline benchmarks |
| | **Waymo Open Dataset** | 100k+ 20-second segments, 20M+ frames | Motion prediction, trajectory forecasting, agent interaction modeling |
| | **Argoverse 2** | 250,000 motion forecasting scenarios | Long-horizon multi-modal trajectory prediction research |
| **Tier C (Synthetic)** | **RoadRunner & CARLA** | High-fidelity Indian digital twins & scenario engine | Rare corner-case generation, sensor fault injection, closed-loop SIL testing |

### Data Preprocessing & Canonical Ontology
All heterogeneous datasets are ingested through a **Data Quality Engine** that enforces:
- Canonical Object Ontology (`Vehicle`, `Pedestrian`, `Motorcycle`, `Bicycle`, `AutoRickshaw`, `Bus`, `Truck`, `Animal`, `Obstacle`, `Pothole`, `RoadBoundary`, `TrafficSignal`).
- Standardized SI Units (meters, seconds, radians, m/s, $\text{m/s}^2$).
- **Sequence-Level Splitting:** Dataset splits (Train / Val / Test) are partitioned by complete geographic sequences (never randomly shuffled by frame) to eliminate data leakage.

---

## 5. Team Ownership, Roles & Collaboration Map

| Team Member | Module Ownership | Core Question | Key Hand-off Outputs |
| :--- | :--- | :--- | :--- |
| **Mohith** | AI Perception + Sensor Simulation + Audio | *"What do I see and hear?"* | `Detection2D`, `Detection3D`, Drivable Masks, Road Boundaries, `AudioEvent` |
| **Hanish** | Localization + SLAM + Mapping | *"Where am I?"* | `EgoState` (Pose, velocity, heading, covariance), `MapOccupancy` |
| **Smruthi** | Sensor Fusion + Tracking + Collision Risk | *"What are objects doing?"* | `TrackState` (Multi-object tracks), `RiskState` (TTC, clearance, uncertainty) |
| **Prediction / World Layer** | 4D World Model & Motion Prediction | *"What happens next?"* | `PredictionState` (Multimodal futures), `WorldState` (Unified 4D scene) |
| **Yashwanth** | Behavior Decision + Adaptive Planning | *"What should we do?"* | `BehaviorCommand` (Maneuver states), `Trajectory` (Time, pose, speed, curvature) |
| **Safety Supervisor** | Independent Deterministic Safety Shield | *"Is this action safe?"* | `SafetyStatus` (Safe/Reject, TTC, clearance, reason, emergency flag) |
| **Meghana** | Vehicle Dynamics + Control | *"How do we execute it?"* | `ControlCommand` (Steering $\delta$, throttle, brake), `VehicleFeedback` |
| **Sireesha** | Simulation, Integration, Testing & Data Registry | *"Does it work reliably?"* | Closed-loop test engine, `ValidationResult`, regression suite, data catalog |

---

## 6. Detailed Interface & Message Contracts

```
[Mohith: Perception] ────────> Detection2D, Detection3D, AudioEvent ────────> [Smruthi: Fusion]
[Hanish: Localization] ─────> EgoState, MapOccupancy ─────────────────────────> [Smruthi, Yashwanth, Meghana]
[Smruthi: Fusion] ───────────> TrackState, RiskState ─────────────────────────> [Prediction & World Model]
[Prediction & World Model] ──> WorldState, PredictionState ────────────────────> [Yashwanth: Planning]
[Yashwanth: Planning] ───────> BehaviorCommand, Trajectory ───────────────────> [Safety Supervisor]
[Safety Supervisor] ─────────> SafetyStatus (Safe / Reject / Emergency) ──────> [Meghana: Control]
[Meghana: Control] ──────────> ControlCommand (Steering, Throttle, Brake) ────> [Sireesha: Simulator]
[Meghana: Control] ──────────> VehicleFeedback (Errors, Slip, Feasibility) ───> [Yashwanth: Planning]
```

- **SI Units:** Metres ($m$), seconds ($s$), radians ($rad$), velocity ($m/s$), acceleration ($m/s^2$), curvature ($1/m$).
- **Uncertainty Propagation:** Every estimated pose/velocity carries a covariance matrix; every learned prediction carries a confidence probability $\in [0, 1]$.
- **Independent Safety Veto:** The Safety Supervisor operates deterministically and independently of AI models, retaining the authority to reject candidate trajectories and force emergency braking.

---

## 7. Mathematical Foundations Across Modules

### A. Dynamic Risk Field $R(x, y, t)$ (Prediction & Planning)
Rather than treating obstacles as static binary occupancy grid cells, the system constructs a time-varying continuous risk field:
$$R(x, y, t) = f\Big(P_{\text{collision}}(x, y, t), \text{TTC}, d_{\text{clearance}}, \sigma_{\text{uncertainty}}, \text{RoadConstraints}\Big)$$

### B. Adaptive Cost Function $J(\tau)$ (Behavior & Planning)
The trajectory optimization cost dynamically adapts its weighting coefficients based on road type, weather, and traffic context:
$$J(\tau) = w_g C_{\text{goal}} + w_r C_{\text{risk}} + w_o C_{\text{obstacle}} + w_b C_{\text{boundary}} + w_c C_{\text{comfort}} + w_v C_{\text{velocity}}$$
- **Dense Market Scenario:** Collision risk $w_r \uparrow\uparrow$, clearance $w_o \uparrow\uparrow$, velocity penalty $w_v \downarrow$ (crawling speed permitted).
- **Monsoon Rain Scenario:** Sensor uncertainty weight $\sigma \uparrow$, braking distance margin $\uparrow$, lateral acceleration comfort threshold $\downarrow$.
- **Mountain Ghat Road:** Curvature penalty $w_c \uparrow\uparrow$, road boundary penalty $w_b \uparrow\uparrow$.

### C. 3-DOF Dynamic Bicycle Model with Tire Saturation (Vehicle Dynamics)
In high-speed maneuvers or slippery Indian road surfaces (wet asphalt $\mu = 0.5$, mud/gravel $\mu = 0.3$), tire slip angles become critical:
$$\alpha_f = \delta - \arctan\left(\frac{v_y + L_f r}{v_x}\right), \quad \alpha_r = -\arctan\left(\frac{v_y - L_r r}{v_x}\right)$$
Tire lateral forces follow a nonlinear saturation model with dynamic vertical load transfer:
$$F_{yf} = \mu F_{zf} \tanh\left(\frac{C_f \alpha_f}{\mu F_{zf}}\right), \quad F_{yr} = \mu F_{zr} \tanh\left(\frac{C_r \alpha_r}{\mu F_{zr}}\right)$$
Equations of motion in vehicle body frame:
$$m (\dot{v}_x - v_y r) = F_x - F_{yf} \sin\delta - F_{\text{drag}} - F_{\text{roll}}$$
$$m (\dot{v}_y + v_x r) = F_{yf} \cos\delta + F_{yr}$$
$$I_z \dot{r} = L_f F_{yf} \cos\delta - L_r F_{yr}$$

### D. Curvilinear Model Predictive Control (MPC)
State error vector: $x_e = [e_y, e_\psi, e_v]^T$ (lateral cross-track error, heading error, speed error).  
Discrete-time error dynamics: $x_e(k+1) = A_k x_e(k) + B_k u(k) + d_k$.  
Optimal control law combines feedforward road curvature preview $\delta_{ff} = \arctan(L \kappa_{\text{ref}})$ with optimal Riccati state feedback:
$$u_{\text{opt}} = u_{ff} - K x_e$$
Subject to actuator rate limits ($|\dot{\delta}| \le 38^\circ/\text{s}, |\dot{a}| \le 4.0\text{ m/s}^3$) and physical saturation limits.

---

## 8. Repository Directory Structure

```
Advitiya_Adaptive_path_planning/
├── config/                         # Vehicle, sensor, and controller configuration
│   ├── vehicle_params.m            # Physical parameters (Mass, Lf, Lr, Iz, Cf, Cr, mu)
│   └── controller_params.m         # Controller gains (Pure Pursuit, Stanley, MPC, PID)
├── interfaces/                     # Inter-module message contracts & validator tools
│   ├── validate_interfaces.m       # Validates Trajectory, EgoState, SafetyStatus
│   ├── create_control_command.m    # Serializer for Simulator control inputs
│   └── create_vehicle_feedback.m   # Serializer for Planner feedback & feasibility
├── data_platform/                  # [CROSS-CUTTING] Data & Intelligence Management
│   ├── registry/                   # Dataset cards, metadata catalogs, and schemas
│   ├── quality_engine/             # Quality scoring, integrity & calibration checks
│   └── failure_database/           # Failure logs, reproduction scripts & regression tags
├── perception/                     # [MOHITH] Camera, LiDAR & Audio Perception
│   └── README.md                   # 2D/3D detection, drivable area & acoustic beamforming
├── localization/                   # [HANISH] Localization, SLAM & Mapping
│   └── README.md                   # GNSS/RTK, EKF fusion, coordinate frames & occupancy
├── fusion/                         # [SMRUTHI] Sensor Fusion, Tracking & Risk
│   └── README.md                   # EKF multi-object tracking, TTC calculation & risk
├── prediction/                     # [PREDICTION & WORLD MODEL]
│   └── README.md                   # Multimodal intent prediction & 4D world model layer
├── planning/                       # [YASHWANTH] Behavior & Adaptive Path Planning
│   └── README.md                   # Stateflow behavior, Hybrid A*, TEB/DWA local planner
├── controllers/                    # [MEGHANA] Vehicle Dynamics & Control
│   ├── pure_pursuit_controller.m   # Geometric lookahead tracking baseline
│   ├── stanley_controller.m        # Front-axle tracking + curvature feedforward baseline
│   ├── longitudinal_controller.m   # Velocity PID + drag compensation + throttle/brake split
│   ├── mpc_controller.m            # Curvilinear Model Predictive Control with DARE solver
│   ├── safety_override.m           # Deterministic emergency braking arbitration
│   └── vehicle_control_executive.m # Master execution manager tying controllers together
├── models/                         # [MEGHANA] Physical Plant & Actuator Simulation
│   ├── kinematic_bicycle.m         # 4-DOF kinematic bicycle model (RK4 integrator)
│   ├── dynamic_bicycle.m           # 3-DOF dynamic bicycle with nonlinear tire saturation
│   └── actuator_model.m            # 1st-order lags, steering rate limiters & jerk bounds
├── simulation/                     # [SIREESHA] Closed-Loop Simulation & Integration
│   └── simulate_closed_loop.m      # Simulation engine driving plant against trajectories
├── simulink/                       # [MEGHANA & SIREESHA] Simulink System Models
│   ├── vehicle_control_system.slx  # Complete Simulink model with scopes & function blocks
│   └── build_simulink_model.m      # Programmatic builder script for .slx model
├── scenarios/                      # Canonical Indian Road Driving Scenarios
│   └── generate_scenarios.m        # Pothole dodge, ghat hairpin, double lane change, cow stop
├── tests/                          # Automated Verification & Benchmark Suites
│   └── run_benchmark.m             # MATLAB automated performance comparison suite
├── python_sim/                     # Standalone Python Verification Harness
│   └── run_simulation.py           # NumPy/SciPy rapid closed-loop simulation runner
├── benchmark_pothole_dodge.png     # Verification plot: 1.5m pothole swerve tracking
├── benchmark_ghat_curve.png        # Verification plot: Mountain ghat hairpin curve tracking
├── benchmark_emergency_stop.png    # Verification plot: Cow/pedestrian emergency stop response
├── benchmark_results.csv           # Quantitative metric log table
└── README.md                       # Master project guide and documentation
```

---

## 9. Benchmark Verification & Performance Metrics

The complete control and dynamics stack was benchmarked across representative Indian road scenarios on the 3-DOF Dynamic Bicycle model:

| Benchmark Scenario | Controller | Max Lateral Error $\|e_y\|$ | RMSE Lateral Error | Tracking Assessment |
| :--- | :--- | :---: | :---: | :---: |
| **Pothole Quick Dodge** | Pure Pursuit | 0.150 m | 0.060 m | Stable, slight corner cut |
| ($1.5$m swerve at 36 km/h) | Stanley | 0.150 m | 0.038 m | High accuracy, smooth recovery |
| | **Curvilinear MPC** | **0.150 m** | **0.029 m** | **Optimal performance (lowest RMS)** |
| **Ghat Road Hairpin Curve** | Pure Pursuit | 0.823 m | 0.270 m | Noticeable inside cutting |
| ($R=18$m bend, 40 to 20 km/h) | Stanley | 0.477 m | 0.249 m | Smooth heading tracking |
| | **Curvilinear MPC** | **0.471 m** | **0.164 m** | **Highest curve tracking precision** |

### Emergency Collision Avoidance Test:
- **Scenario:** Unexpected cow / pedestrian crossing triggered at $t = 3.5$s while cruising at $43\text{ km/h}$.
- **Result:** Safety supervisor decouples planner, commands full emergency deceleration ($a = -8.5\text{ m/s}^2$), and brings the vehicle to a full stop in **$1.55$ seconds** with **zero lateral deviation** and **zero tire lock-up**.

---

## 10. How to Run and Test

### A. In MATLAB / Simulink
1. Launch MATLAB and set Current Folder to the project root:
   ```matlab
   addpath(genpath('.'));
   ```
2. Run the automated multi-controller benchmark suite:
   ```matlab
   run_benchmark;
   ```
3. Open or build the Simulink model programmatically:
   ```matlab
   build_simulink_model;
   open_system('vehicle_control_system');
   ```

### B. In Standalone Python Environment (No MATLAB license required)
```bash
python -u python_sim/run_simulation.py
```
This executes the 3-DOF dynamic bicycle simulation, verifies tracking errors, and updates the benchmark comparison plots.

---

## 11. Git Branching Strategy & Contribution Workflow

```
main (stable milestones & demonstration baselines)
  │
develop (team integration branch)
  ├── feature/mohith-perception
  ├── feature/hanish-localization
  ├── feature/smruthi-fusion
  ├── feature/yashwanth-planning
  ├── feature/meghana-control
  └── feature/sireesha-simulation
```

1. **Feature Branch:** Each team member develops on their designated branch off `develop`.
2. **Commit Standard:** Commit messages follow Conventional Commits (`feat:`, `fix:`, `test:`, `docs:`).
3. **Interface Agreement:** Any change to schemas in `interfaces/` requires consumer and producer consensus before merging.
4. **Regression Gate:** Every PR merged into `develop` must pass the canonical scenario benchmark suite with zero collisions.

---

## 12. Team Mantra
* **Mohith** = SEE & HEAR
* **Hanish** = LOCATE & MAP
* **Smruthi** = FUSE, TRACK & ASSESS
* **Prediction & World Layer** = ANTICIPATE & REPRESENT
* **Yashwanth** = DECIDE & PLAN
* **Safety Supervisor** = REJECT UNSAFE ACTIONS
* **Meghana** = DYNAMICS & CONTROL
* **Sireesha** = SIMULATE, INTEGRATE & VALIDATE
"# SIH" 
