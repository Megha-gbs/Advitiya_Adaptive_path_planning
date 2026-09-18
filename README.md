# SIH Problem Statement 26037: Vehicle Dynamics & Control
## Module Owner: Meghana
**Project Title:** Adaptive Path Planning and Collision Avoidance for Autonomous Vehicles on Unstructured Indian Roads  
**Core Responsibility:** *"How do we execute it?"* — Transform planner trajectories into physically realizable steering, throttle, and brake commands while respecting tire friction limits, actuator constraints, and emergency safety interventions.

---

## 1. System Architecture & Information Flow

```
                      +-----------------------------+
                      |   Hanish (Localization)     |
                      |          EgoState           |
                      +--------------+--------------+
                                     |
+--------------------------+         |         +---------------------------+
|  Yashwanth (Planning)    |         |         |     Safety Supervisor     |
|       Trajectory         |         |         |       SafetyStatus        |
+------------+-------------+         |         +-------------+-------------+
             |                       |                       |
             +-------------------->  v  <--------------------+
                      +-----------------------------+
                      |       MEGHANA (OWNER)       |
                      | Vehicle Dynamics & Control  |
                      |  - Pure Pursuit (Baseline)  |
                      |  - Stanley (Baseline)       |
                      |  - Curvilinear MPC (Adv.)   |
                      |  - Longitudinal Splitter    |
                      |  - Safety Override Logic    |
                      +--------------+--------------+
                                     |
             +-----------------------+-----------------------+
             |                                               |
             v                                               v
+-----------------------------+               +-----------------------------+
|    Sireesha (Simulator)     |               |    Yashwanth (Planning)     |
|       ControlCommand        |               |       VehicleFeedback       |
| (steering, throttle, brake) |               | (feasibility, errors, slip) |
+-----------------------------+               +-----------------------------+
```

---

## 2. Directory Structure

```
Matlab-simu/
├── config/
│   ├── vehicle_params.m            # Tata Nexon/Swift vehicle inertia & tire params
│   └── controller_params.m         # Pure pursuit, Stanley, PID & MPC gains
├── models/
│   ├── kinematic_bicycle.m         # 4-DOF kinematic bicycle model with RK4
│   ├── dynamic_bicycle.m           # 3-DOF dynamic bicycle with nonlinear tire saturation
│   └── actuator_model.m            # 1st-order actuator lag & slew rate limiters
├── controllers/
│   ├── pure_pursuit_controller.m   # Geometric lookahead tracking
│   ├── stanley_controller.m        # Front-axle tracking + curvature feedforward
│   ├── longitudinal_controller.m   # Velocity PID + aero/rolling drag feedforward
│   ├── mpc_controller.m            # Constrained Model Predictive Control
│   ├── safety_override.m           # Deterministic emergency braking override
│   └── vehicle_control_executive.m # Master execution wrapper
├── interfaces/
│   ├── validate_interfaces.m       # Validates input structs from team
│   ├── create_control_command.m    # Packs commands to simulator
│   └── create_vehicle_feedback.m   # Packs telemetry feedback to planner
├── scenarios/
│   └── generate_scenarios.m        # Indian road edge cases (pothole dodge, ghat hairpin, etc.)
├── simulink/
│   └── build_simulink_model.m      # Programmatic builder for vehicle_control_system.slx
├── tests/
│   └── run_benchmark.m             # MATLAB automated benchmark runner
├── python_sim/
│   └── run_simulation.py           # Standalone Python verification suite
├── benchmark_pothole_dodge.png     # Validation figure: Pothole avoidance
├── benchmark_ghat_curve.png        # Validation figure: Mountain hairpin bend
├── benchmark_emergency_stop.png    # Validation figure: Cow/pedestrian emergency brake
└── README.md
```

---

## 3. Team Interface Contracts

### Hand-off 1: Input from Yashwanth (Behavior & Adaptive Planning)
* **Message:** `Trajectory`
* **Contents:**
  * `time`: 1xN timestamps (s)
  * `x`, `y`: 1xN global coordinates (m)
  * `yaw`: 1xN heading angle (rad)
  * `velocity`: 1xN reference speeds (m/s)
  * `acceleration`: 1xN reference accelerations ($m/s^2$)
  * `curvature`: 1xN path curvature $\kappa$ ($1/m$)

### Hand-off 2: Input from Hanish (Localization & Mapping)
* **Message:** `EgoState`
* **Contents:**
  * `timestamp`: current timestamp (s)
  * `pose`: $[X, Y, Z]$ global position (m)
  * `velocity`: $[v_x, v_y, v_z]$ linear velocity (m/s)
  * `heading`: yaw $\psi$ (rad)
  * `covariance`: estimation uncertainty matrix

### Hand-off 3: Input from Safety Supervisor
* **Message:** `SafetyStatus`
* **Contents:**
  * `safe`: `'safe'` | `'reject'` | `'unsafe'`
  * `TTC`: Time-To-Collision (s)
  * `clearance`: Distance to nearest critical obstacle (m)
  * `reason`: Diagnostic string
  * `emergency_flag`: Boolean (true triggers instant full emergency braking)

### Hand-off 4: Output to Sireesha (Simulation & Integration)
* **Message:** `ControlCommand`
* **Contents:**
  * `timestamp`: Command timestamp (s)
  * `steering`: Front wheel steering angle $\delta$ (rad), bounded in $[-32^\circ, +32^\circ]$
  * `throttle`: Normalized engine torque request $[0.0, 1.0]$
  * `brake`: Normalized brake pressure request $[0.0, 1.0]$
  * `accel_cmd`: Demanded longitudinal acceleration ($m/s^2$)

### Hand-off 5: Feedback to Yashwanth (Planning Loop)
* **Message:** `VehicleFeedback`
* **Contents:**
  * `cross_track_err`: Lateral error $e_y$ (m)
  * `heading_err`: Heading error $e_\psi$ (rad)
  * `speed_err`: Speed error $e_v$ (m/s)
  * `lat_accel`: Current lateral acceleration $a_y = v^2 \kappa$ ($m/s^2$)
  * `slip_angle_f`, `slip_angle_r`: Front/rear tire slip angles (rad)
  * `feasibility_code`: 
    * `0: OK`
    * `1: TRACTION_LIMIT_APPROACHING` ($a_y > 0.85 \mu g$)
    * `2: HIGH_TRACKING_ERROR`
    * `3: STEERING_SATURATED`
    * `4: SAFETY_OVERRIDE_ACTIVE`

---

## 4. Benchmark Performance on Indian Scenarios

| Scenario | Controller | Max Lateral Error $\|e_y\|$ | RMSE Lateral Error | Tracking Accuracy |
| :--- | :--- | :---: | :---: | :---: |
| **Pothole Quick Dodge** | Pure Pursuit | 0.150 m | 0.060 m | Baseline Pass |
| ($1.5$m swerve at 36 km/h) | Stanley | 0.150 m | 0.038 m | High Precision |
| | **Curvilinear MPC** | **0.150 m** | **0.029 m** | **Best Performance** |
| **Ghat Road Hairpin** | Pure Pursuit | 0.823 m | 0.270 m | Noticeable Corner Cut |
| ($R=18$m bend, 40 to 20 km/h) | Stanley | 0.477 m | 0.249 m | Good Recovery |
| | **Curvilinear MPC** | **0.471 m** | **0.164 m** | **Best Smoothness & Accuracy** |

---

## 5. How to Run

### In MATLAB:
1. Open MATLAB and navigate to this directory: `c:\Users\megha\OneDrive\Pictures\Desktop\Matlab-simu`
2. Run the benchmark suite:
   ```matlab
   addpath(genpath('.'));
   run_benchmark;
   ```
3. To generate or open the Simulink block diagram:
   ```matlab
   build_simulink_model;
   open_system('vehicle_control_system');
   ```

### In Python (Instant verification without MATLAB license):
```bash
python -u python_sim/run_simulation.py
```
This runs the closed-loop 3-DOF dynamic bicycle simulation, calculates all metrics, and updates the benchmark comparison plots.
