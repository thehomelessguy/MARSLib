---
name: marslib-telemetry
description: Helps manage logging and telemetry in MARSLib using AdvantageKit and AdvantageScope. Use when adding new log keys, debugging Sim/Real disparities, or enforcing replay-only algorithm constraints.
---

# MARSLib Telemetry & Logging Skill

You are a telemetry engineer for Team MARS 2614. When modifying logging or configuring AdvantageScope:

## 1. Architecture

MARSLib uses AdvantageKit for all telemetry, enforcing deterministic log-replay:

| Component | Purpose |
|---|---|
| `@AutoLog` inner class | Auto-generates logging for IO inputs (sensors, encoders) |
| `Logger.processInputs()` | Records IO inputs every periodic cycle |
| `Logger.recordOutput()` | Records algorithm outputs (setpoints, decisions, poses) |
| `LoggedTunableNumber` | Runtime-tunable constants with automatic logging |
| `.wpilog` files | Binary log format for post-match replay |

### Data Flow
```
Hardware → IO.updateInputs() → Logger.processInputs() → [Logged as Inputs]
                                    ↓
              Subsystem.periodic() algorithms → Logger.recordOutput() → [Logged as Outputs]
```

**Inputs** are recorded for replay. **Outputs** are recomputed from inputs during replay. This separation is what makes deterministic replay possible.

## 2. Key Rules

### Rule A: Never Use SmartDashboard or Raw NetworkTables
All telemetry MUST go through AdvantageKit's `Logger`. Do NOT use `SmartDashboard.putNumber()` or `NetworkTableInstance.getDefault().getTable()`. These bypass the replay pipeline and create non-deterministic behavior.

### Rule B: IO Inputs vs Algorithm Outputs
- **Inputs** (`@AutoLog` in IO classes): Sensor readings, encoder positions, current draw. These are the "truth" that gets replayed.
- **Outputs** (`Logger.recordOutput()` in subsystems): Setpoints, decisions, computed poses. These are recomputed from inputs during replay.

Never put algorithm decisions in IO inputs. Never put sensor readings in outputs. Mixing them breaks replay determinism.

### Rule C: Never Branch on Replay State
Do NOT write `if (!isReplay()) { ... }`. The algorithm logic in `periodic()` must execute identically whether running live or replaying from a log. The ONLY layer that diverges is the IO implementation (real hardware vs. sim vs. replay).

### Rule D: Match Units Between Sim and Real
The `IOSim` and `IOReal` implementations MUST use identical units. If `IOReal` reports position in radians, `IOSim` must also report radians — not rotations, not degrees. Unit mismatches are the #1 cause of "works in sim, fails on robot" bugs.

## 3. Adding New Log Keys

1. For sensor data: add the field to the IO's `@AutoLog` inner class.
2. For algorithm outputs: add `Logger.recordOutput("Subsystem/KeyName", value)` in the subsystem's `periodic()`.
3. Use nested paths: `"Superstructure/CollisionClamp"` not `"collision_clamp"`.
4. Log structured types when possible: `Pose2d`, `SwerveModuleState[]`, `Mechanism2d`.
5. Update the relevant skill's Telemetry section with the new key.

## 4. AdvantageScope Layout Configuration
When building AdvantageScope layouts:
- Use paths matching `Logger.recordOutput()` keys (e.g., `RealOutputs/Swerve/Pose`)
- Build 3D field visualizations using `Pose2d[]` arrays
- Build swerve visualizations using `SwerveModuleState[]` arrays
- See the `advantagescope-layouts` skill for MCP tool usage

## 5. Telemetry (Meta)
These are the telemetry keys logged by the telemetry system itself:
- `Logger/Timestamp` — Current FPGA timestamp
- `Logger/LoopCycleMs` — Time taken for one full periodic cycle
- `Logger/LogSizeBytes` — Cumulative log file size
- `BuildConstants/Version` — Deployed code version
- `BuildConstants/GitSHA` — Git commit hash
