# Module 9: Behavior Decision + Adaptive Path Planning
**Owner: Yashwanth**

## Objectives
- Stateflow behavior decision making: `DRIVING, FOLLOW, STOP, YIELD, AVOID, OVERTAKE, MERGE, WAIT, PARK, EMERGENCY_BRAKE, RECOVER, SAFE_DEGRADED`.
- Global path generation (A* and kinematic Hybrid A*).
- Dynamic local trajectory generation (TEB / DWA) with continuous replanning around potholes and informal road obstacles.

## Outputs Handed Off
- `BehaviorCommand` -> Planner / Safety Supervisor
- `Trajectory` -> Safety Supervisor / Meghana (Vehicle Dynamics & Control)
