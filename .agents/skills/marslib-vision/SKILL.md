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
    → Scale stdDevs by distance and velocity
    → SwerveDrive.addVisionMeasurement(pose, timestamp, stdDevs)
```

## 2. Key Rules

### Rule A: Scale Standard Deviations Dynamically
Do NOT use fixed standard deviations. Scale them exponentially based on:
- **Distance to nearest tag** — farther = less trust
- **Robot velocity** — faster = more motion blur = less trust
- **Ambiguity metric** — higher = more uncertain = less trust
```java
double stdDev = BASE_STD_DEV * Math.exp(DISTANCE_SCALE * distance) * (1 + velocity * VEL_SCALE);
```

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

## 3. Adding New Camera Sources

1. Create a new IO implementation (e.g., `AprilTagVisionIOCustom.java`) implementing `AprilTagVisionIO`.
2. In `updateInputs()`, populate: `poses`, `timestamps`, `ambiguities`, `tagCount`.
3. Wire it in `RobotContainer`:
   ```java
   MARSVision vision = new MARSVision(
       new AprilTagVisionIOPhoton("FrontCam", cameraPose),
       new AprilTagVisionIOPhoton("RearCam", cameraPose2)
   );
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
- `Vision/CameraCount` — Number of active camera sources
- `Vision/TagsDetected` — Total AprilTags seen this frame
- `Vision/EstimatedPose` — Latest fused Pose2d from vision
- `Vision/AmbiguityRejections` — Count of rejected high-ambiguity measurements
- `Vision/AverageLatencyMs` — Mean pipeline latency across all cameras
- `Vision/StdDevX`, `StdDevY`, `StdDevTheta` — Current dynamic standard deviations
- `Vision/CamerasConnected` — Boolean[] per camera
