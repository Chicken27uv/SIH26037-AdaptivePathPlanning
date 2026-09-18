# SIH 26037 — Common Interface Specification

## Purpose

This document defines the shared data contracts between M1-M8.

All modules must follow these interfaces during integration.

MATLAB version:
R2026a

---

# Coordinate Convention

World-frame convention used by this project:

- Position: `[x y]` in metres
- Velocity: `[vx vy]` in metres/second
- Heading: radians
- Time: seconds

Trajectory/path format:

`N x 2 [x y]`

---

# Ego State

MATLAB structure:

ego.position
ego.velocity
ego.heading
ego.speed
ego.timestamp

Definitions:

- position: `[x y]`, m
- velocity: `[vx vy]`, m/s
- heading: rad
- speed: m/s
- timestamp: s

---

# Object Representation

MATLAB structure:

object.id
object.class
object.position
object.velocity
object.heading
object.timestamp

Position:

`[x y]`, m

Velocity:

`[vx vy]`, m/s

Heading:

rad

---

# M1 — Scenario & Road Environment

### Produces

Scenario/environment representation.

Minimum information:

- road/drivable-space representation
- static obstacles
- dynamic actors
- scenario name
- actor ground truth

### Required scenarios

1. VillageRoad
2. UrbanIntersection
3. HighwayMerge
4. MarketArea
5. CattleCrossing

M1 must provide at least two detailed RoadRunner scenes:

- unmarked village road
- busy urban intersection without signals

---

# M2 — Multi-Sensor Simulation & Fusion

### Input

M1 environment/scenario.

### Produces

Sensor outputs and fused environment representation.

Sensors:

- camera
- LiDAR
- radar

Fused objects must use the common Object Representation.

Conceptual output:

sensorData.camera
sensorData.lidar
sensorData.radar
sensorData.fusedObjects

---

# M3 — Perception & Object Tracking

### Input

Sensor/fused detections from M2.

### Produces

Tracked objects.

Output:

trackedObjects

Each object must contain:

- id
- class
- position
- velocity
- heading
- timestamp

Tracking history may additionally be stored.

---

# M4 — Short-Term Motion Prediction

### Input

trackedObjects

### Produces

Predicted short-term trajectories.

Each prediction should contain:

prediction.id
prediction.class
prediction.currentPosition
prediction.futureTrajectory
prediction.timeVector

futureTrajectory:

`N x 2 [x y]`

The prediction module must support irregular/non-lane-based movement.

Examples:

- pedestrian crossing
- cattle crossing
- vehicle merging
- curved/irregular movement

---

# M5 — Decision & Behavior Logic

### Input

- ego state
- tracked objects
- predicted trajectories
- road/environment state

### Produces

Behavior/decision command.

Example:

decision.state
decision.action
decision.targetSpeed
decision.replanRequired

Possible actions:

- CRUISE
- FOLLOW
- SLOW
- STOP
- AVOID
- REPLAN

---

# M6 — Adaptive Path Planning

### Input

- ego state
- drivable-space/road representation
- obstacles
- predicted trajectories
- behavior decision

### Produces

planned path.

Minimum output:

plannedPath

Format:

`N x 2 [x y]`

Additional outputs:

planningTime
replanningLatency
replanCount
pathLength
pathSmoothness
minimumClearance

M6 must support adaptive replanning when the current path becomes blocked.

---

# M7 — Vehicle Motion & Control

### Input

- ego state
- target/planned path
- target speed

### Produces

vehicle control and actual vehicle trajectory.

Outputs may include:

control.steering
control.speed
control.acceleration

vehicleState

actualTrajectory

trackingError

---

# M8 — Closed-Loop Validation & Metrics

### Input

Outputs from M1-M7.

### Produces

scenario-level validation results.

Required metrics include:

- collision status
- minimum clearance
- replanning latency
- path smoothness
- scenario completion rate
- replan count

Additional useful metrics:

- planning time
- path length
- vehicle tracking error
- maximum/average steering
- scenario duration

---

# Integration Rule

Modules must be independently runnable before integration.

During independent development, modules may use:

- synthetic test data
- built-in MATLAB examples
- simple generated scenarios

During final integration, real outputs from upstream modules must replace test data.

---

# Development Rules

1. Do not modify another member's module.
2. Do not change Common interfaces without team agreement.
3. Do not change field names casually.
4. Use SI units defined above.
5. Use `[x y]` for positions and trajectories.
6. Do not hard-code final validation metrics.
7. All reported metrics must come from simulation.
8. Keep module-specific helper functions inside the module folder.
9. Common contains only shared interfaces/configuration/test utilities.
10. Every module must have a standalone demo/test script.

---

# Integration Flow

M1
→ M2
→ M3
→ M4
→ M5
→ M6
→ M7
→ M8

M8 is the final validation layer.

M6 must remain capable of demonstrating path planning/replanning independently before full closed-loop integration.