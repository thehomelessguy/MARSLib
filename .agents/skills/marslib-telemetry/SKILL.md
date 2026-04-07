---
name: marslib-telemetry
description: Helps manage logging and telemetry in MARSLib using AdvantageKit and AdvantageScope. Use when adding new structural layouts, debugging Sim/Real disparities, or enforcing replay-only algorithm constraints.
---

# MARSLib Telemetry & Logging Skill

You are an expert FRC Software Developer. When asked to modify logging, parse WPILog files offline, or configure AdvantageScope layouts for Team MARS 2614, follow these rules:

## 1. Native AdvantageKit Constraint
- All telemetry MUST be processed through the AdvantageKit `Logger` pattern.
- You may NOT use `SmartDashboard` or `NetworkTables` directly for standard sensor outputs. Use `@AutoLog` inner classes for `[Name]IO` modules and `Logger.recordOutput("Path", value)` for subsystem logic.

## 2. Replay & Disparity Resolution
When debugging disparities between `Real` and `Sim` behavior from logs:
- **Never Change Replay Logic:** The core algorithms in the `periodic()` loops must execute identically whether running live or replaying from a `.wpilog`. Do not wrap algorithms in `if (!isReplay())`.
- **Hunts in Real vs Sim:** The only layer that diverges is the `[Name]IO.java` implementation. Ensure that `IOSim` mathematically updates fields with the same unit scales (e.g. `Radians` vs `Rotations`) as the `IOReal` hardware implementation.

## 3. AdvantageScope Layout Configurations
When the user asks you to modify the `advantagescope_layout.json`:
- Use the overarching AdvantageScope knowledgebase for valid layout schema structures.
- Use native paths like `RealOutputs/SubsystemName/...` to match what `Logger.recordOutput` emits.
- Build structural visualizations (3D lines, Odometry ghosts, Swerve Vectors) using arrays of `Pose2d` or `SwerveModuleState` structs natively pumped to the 3D tabs.
