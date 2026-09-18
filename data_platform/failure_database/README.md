# Failure Database & Regression Suite
**Owner: Sireesha & Cross-Functional Team**

## Purpose
Captures closed-loop simulation failures, collision events, safety supervisor interventions, and near-misses for continuous regression testing and model fine-tuning.

## Schema
- `FailureID`: Unique failure identifier (e.g. `FAIL_MOTO_CUTIN_042`)
- `ScenarioID`: Canonical scenario configuration identifier
- `ModelVersion`: Git commit hash & model checkpoint version
- `Timestamp`: Exact time step when failure or safety override occurred
- `SensorState`: Sensor confidence, weather, and occlusions present
- `PredictedIntent`: Predicted future trajectories of obstacle vs actual motion
- `PlannerDecision`: Maneuver state selected by Stateflow / local planner
- `ControlResponse`: Actuator commands generated prior to failure
- `RootCauseModule`: `PERCEPTION | FUSION | PREDICTION | PLANNING | CONTROL | SIMULATION`
- `SyntheticVariations`: List of generated edge-case synthetic scenarios for retraining
