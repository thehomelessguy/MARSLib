---
name: marslib-vision
description: Helps implement or debug Vision localization, Megatag fusion, and origin coordinate spaces in MARSLib.
---

# MARSLib Vision & Odometry Skill

You are an expert FRC Software Developer. When adding PhotonVision, Limelight targeting, or multi-camera fusion to Team MARS 2614's architecture:

## 1. Sensor Fusion Strategy
- **Standard Deviations:** Do not blindly trust camera inputs. Standard deviations inside Odometry updates MUST exponentially back-off based on target distance, velocity, and ambiguity metrics.
- **Latency Backlogging:** Use `addVisionMeasurement()` effectively by feeding it the exact timestamp the image was captured, NOT the current timestamp. Let the Swerve Odometry buffer mathematically reconstruct the past pose.

## 2. Alliance Origin Management
- Standardize all coordinates to **Blue Alliance Origin** on the field.
- When generating poses specifically for Red Alliance paths or targets, mirror coordinates relative to the halfway Y/X intercepts depending on the game layout.
- The `AprilTagVisionIOSim` must natively report localized Blue Alliance coordinates based on perfectly projected `dyn4j` field geometry matrices.

## 3. Stochastic Fidelity in Simulation
- `AprilTagVisionIOSim` should intentionally drop frames randomly (e.g. 5% packet loss) to accurately simulate real-world motion blur or network stutter.
- Introduce Gaussian noise vectors to `Pose3d` outputs based on the robot's physical simulation bounds and velocity vectors.
