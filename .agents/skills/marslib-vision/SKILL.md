---
name: marslib-vision
description: Helps implement or debug Vision localization, Megatag fusion, and origin coordinate spaces in MARSLib. Use when injecting AprilTag 3D Odometry poses, configuring VisionIO interfaces, or mapping vision data properly into the AdvantageScope 3D field layout.
---

# MARSLib Vision & Odometry Skill

You are a vision systems engineer for Team MARS 2614. When adding cameras, tuning pose estimation, or debugging vision fusion:

## 1. Architecture

The vision system lives in `com.marslib.vision` with 8 classes:

| Class | Purpose |
|---|---|
| `MARSVision` | Top-level subsystem — multi-camera fusion, standard deviation scaling, filtering, pose injection |
| `AprilTagVisionIO` | IO interface with `@AutoLog` inputs (poses, ambiguity, timestamps) |
| `AprilTagVisionIOPhoton` | PhotonVision camera implementation |
| `AprilTagVisionIOLimelight` | Limelight camera implementation |
| `AprilTagVisionIOSim` | Simulated camera using `VisionSystemSim` from PhotonLib |
| `VIOSlamIO` | IO interface for VIO SLAM odometry sources |
| `VIOSlamIOQuestNav` | Meta Quest motion tracking implementation |
| `VIOSlamIOROS2` | ROS2-based SLAM implementation |

### Fusion Pipeline
```
Camera → AprilTagVisionIO.updateInputs() → MARSVision.periodic()
    → Filter by Z-Height (reject "flying robot")
    → Filter by Field Bounds (reject if coordinates are outside margin)
    → Filter by Tilt (reject if beached/riding a bump > 15°)
    → Filter by Yaw Rate (reject if spinning > 120°/s due to motion blur)
    → Filter by Ambiguity (reject > threshold for PhotonVision, Limelight defaults to 0)
    → Scale stdDevs quadratically by distance
    → Multi-tag boost (tighten stdDevs when >1 tag visible)
    → SwerveDrive.addVisionMeasurement(pose, timestamp, stdDevs)
```

## 2. Key Rules

### Rule A: Scale Standard Deviations Quadratically by Distance
Do NOT use fixed standard deviations. Scale them **quadratically** based on distance to nearest tag:
```java
// Quadratic: further = exponentially less trust (squared)
double linearStdDev = Constants.VisionConstants.TAG_STD_BASE * Math.pow(avgDist, 2);

// MegaTag2 boost: dramatically tighten when multiple tags are visible
if (tagCount > 1) {
    linearStdDev *= Constants.VisionConstants.MULTI_TAG_STD_MULTIPLIER; // 0.1x
}

// Angular stdDev derived from linear
double angularStdDev = linearStdDev * Constants.VisionConstants.ANGULAR_STD_MULTIPLIER;
```

> **NOTE:** The scaling is `Math.pow(distance, 2)` (quadratic), NOT exponential. There is NO velocity-based scaling — only distance and tag count affect standard deviations.

### Rule B: Use Capture Timestamp, Not Current Time
Feed `addVisionMeasurement()` the exact FPGA timestamp when the image was captured, NOT `Timer.getFPGATimestamp()`. The pose estimator's internal buffer reconstructs the past to fuse the measurement at the correct historical pose.

### Rule C: Standardize to Blue Alliance Origin
All coordinates MUST be in Blue Alliance origin (0,0 at blue driver station corner). When generating Red Alliance paths, mirror coordinates. `AprilTagVisionIOSim` MUST report Blue Alliance coordinates matching the field layout.

### Rule D: VisionSystemSim is Static — Reset in Tests
`AprilTagVisionIOSim` uses a static `VisionSystemSim` field. Between tests:
```java
AprilTagVisionIOSim.resetSimulation();
```
Without this, camera configurations leak across test classes, producing stale pose estimates.

### Rule E: Simulate Imperfection
`AprilTagVisionIOSim` MUST inject:
- Gaussian noise on Pose3d outputs (proportional to distance and velocity)
- Stochastic frame dropping (~5% packet loss rate)
- Latency jitter (±10ms)

Without these, sim performs unrealistically well and hides real-world vision issues.

### Rule F: Rejection Filters (254-Style)
Five strict rejection filters are applied before any pose is accepted:
1. **Z-height check** — observations with |z| > `MAX_Z_HEIGHT` (0.5m) are rejected as hallucinations.
2. **Field bounds check** — observations outside field dimensions + `FIELD_MARGIN_METERS` (0.5m) are rejected.
3. **Beached/Tilt check** — observations where max(abs(pitch), abs(roll)) > `MAX_TILT_DEG` (15°) are rejected because the PnP geometry assumptions are violated.
4. **Yaw Rate check** — observations where yaw velocity > `MAX_YAW_RATE_DEG_PER_SEC` (120°/s) are rejected due to motion blur and rolling shutter warp.
5. **Ambiguity check** — single-tag observations with ambiguity > `MAX_AMBIGUITY` (0.2) are rejected. (Limelight MegaTag2 defaults to 0).

## 3. Adding New Camera Sources

1. Create a new IO implementation (e.g., `AprilTagVisionIOCustom.java`) implementing `AprilTagVisionIO`.
2. In `updateInputs()`, populate: `estimatedPoses`, `timestamps`, `ambiguities`, `averageDistancesMeters`, `tagCounts`.
3. Wire it in `RobotContainer` by passing the classes to the `MARSVision` constructor.
4. `MARSVision` handles multi-camera aggregation naturally.

## 4. Command API
Vision is passive — no manual commands needed. The subsystem automatically injects measurements into `SwerveDrive` every periodic cycle.

## 5. Telemetry
The following log keys are written by `MARSVision.periodic()`:

| Key | Description |
|---|---|
| `Vision/AprilTag/{i}` | Raw `@AutoLog` inputs for camera index `i` |
| `Vision/ValidPoses/{i}` | Accepted Pose2d after all filtering for camera `i` |
| `Vision/Rejected/Tilt/{i}` | Boolean: true if pose rejected due to robot tilt > 15° |
| `Vision/Rejected/YawRate/{i}` | Boolean: true if pose rejected due to extreme spinning |
| `Vision/Rejected/OutOfBounds/{i}` | Boolean: true if hallucinated pose is outside arena |
| `Vision/Rejected/ZHeight/{i}` | Boolean: true if pose rejected due to flying robot Z hallucination |
| `Vision/AcceptedCount/{i}` | Raw counts of approved poses per cycle |

### Constants Reference (`Constants.VisionConstants`)
| Field | Default | Purpose |
|---|---|---|
| `TAG_STD_BASE` | 0.05 | Base linear stdDev for single-tag |
| `MAX_AMBIGUITY` | 0.2 | Rejection threshold for ambiguous single-tag |
| `MAX_Z_HEIGHT` | 0.5m | Rejection threshold for hallucinated poses |
| `MAX_TILT_DEG` | 15.0° | Rejection threshold for pitch/roll beaching |
| `MAX_YAW_RATE_DEG_PER_SEC` | 120°/s | Rejection threshold for motion blur spinning |
| `FIELD_MARGIN_METERS` | 0.5m | Rejection threshold for bounds checking |
| `MULTI_TAG_STD_MULTIPLIER` | 0.1 | StdDev reduction factor when >1 tag visible |
| `ANGULAR_STD_MULTIPLIER` | 2.0 | Linear → angular stdDev conversion factor |

## 6. Elite References
When extracting elite vision logic (like Megatag 2 or multi-camera fusion) or debugging underlying behavior:
*   **Photonviz / Limelight documentation** (MegaTag2)
*   **Team 254 (Cheesy Poofs)** — Reference for strict rejection filters (yaw rate, bounds, tilt).
