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
| `MARSVision` | Top-level subsystem — multi-camera fusion, standard deviation scaling, pose injection |
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
    → Filter by ambiguity (reject > threshold)
    → Filter by Z-height (reject "flying robot" hallucinations)
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

### Rule F: Rejection Filters
Three rejection filters are applied before any pose is accepted:
1. **Ambiguity check** — single-tag observations with ambiguity > `MAX_AMBIGUITY` (0.2) are rejected.
2. **Z-height check** — single-tag observations with |z| > `MAX_Z_HEIGHT` (0.5m) are rejected as hallucinations.
3. **Distance cutoff** (B.R.E.A.D. 2025) — single-tag observations beyond `MAX_TAG_DISTANCE` (3.0m) are rejected. At long range, single AprilTag solutions are geometrically degenerate, so hard-cutting prevents large-amplitude pose jumps.

Multi-tag observations (tagCount > 1) bypass all three filters.

### Rule G: Sigmoid Distance Weight
The standard deviation is further scaled by a sigmoid-based weight function that smoothly reduces trust as distance increases:
```java
// Weight = baseWeight (capped at MIN_WEIGHT at FAR_DISTANCE)
// Beyond FAR_DISTANCE: exponential halving penalty every 2m
double baseWeight = MIN_WEIGHT + (1.0 - MIN_WEIGHT)
    * (1.0 / (1.0 + Math.exp((dist - CLOSE_DISTANCE) / (FAR_DISTANCE - CLOSE_DISTANCE) * 4.0)));
```
Key parameters: `CLOSE_DISTANCE=2.0m`, `FAR_DISTANCE=6.0m`, `MIN_WEIGHT=0.1`.

## 3. Adding New Camera Sources

1. Create a new IO implementation (e.g., `AprilTagVisionIOCustom.java`) implementing `AprilTagVisionIO`.
2. In `updateInputs()`, populate: `estimatedPoses`, `timestamps`, `ambiguities`, `averageDistancesMeters`, `tagCounts`.
3. Wire it in `RobotContainer`:
   ```java
   MARSVision vision = new MARSVision(
       swerveDrive,
       java.util.List.of(
           new AprilTagVisionIOPhoton("FrontCam", cameraPose),
           new AprilTagVisionIOPhoton("RearCam", cameraPose2)),
       java.util.List.of()); // Optional: SLAM IOs
   ```
4. `MARSVision` handles multi-camera aggregation automatically — just pass all IO instances.
5. Tune per-camera stdDev scaling in `Constants.VisionConstants`.

## 4. Command API
Vision is passive — no manual commands needed. The subsystem automatically injects measurements into `SwerveDrive` every periodic cycle.

For manual recalibration:
```java
// Reset pose estimator to a known position (e.g., at match start)
drive.resetPose(new Pose2d(startX, startY, startRotation));
```

## 5. Telemetry
The following log keys are written by `MARSVision.periodic()`:

| Key | Description |
|---|---|
| `Vision/AprilTag/{i}` | Raw `@AutoLog` inputs for camera index `i` |
| `Vision/ValidPoses/{i}` | Accepted Pose2d after ambiguity/z-height filtering for camera `i` |
| `Vision/SLAM/{i}` | Raw `@AutoLog` inputs for SLAM source index `i` |
| `Vision/SlamPoses/{i}` | Accepted Pose2d from SLAM source `i` |

### Constants Reference (`Constants.VisionConstants`)
| Field | Default | Purpose |
|---|---|---|
| `TAG_STD_BASE` | 0.05 | Base linear stdDev for single-tag |
| `MAX_AMBIGUITY` | 0.2 | Rejection threshold for ambiguous single-tag |
| `MAX_Z_HEIGHT` | 0.5m | Rejection threshold for hallucinated poses |
| `MULTI_TAG_STD_MULTIPLIER` | 0.1 | StdDev reduction factor when >1 tag visible |
| `ANGULAR_STD_MULTIPLIER` | 2.0 | Linear → angular stdDev conversion factor |
| `SLAM_STD_DEV` | 0.01 | Static stdDev for VIO SLAM measurements |
| `SLAM_ANGULAR_STD_DEV` | 0.5 | Static angular stdDev for VIO SLAM measurements |
| `MAX_TAG_DISTANCE` | 3.0m | Hard cutoff for single-tag distance (B.R.E.A.D.) |

## 6. Elite References (PhotonVision)

When extracting elite vision logic (like Megatag 2 or multi-camera fusion) or debugging underlying behavior, use the core PhotonVision repository as the definitive reference.

*   **PhotonVision Core Vision Engine:** `https://github.com/PhotonVision/photonvision`
