---
name: marslib-control-theory
description: Helps architect PID loops, Feedforwards, slew limiters, and kinematics in MARSLib mechanisms. Use when tuning mechanisms, implementing control loops, or working with swerve dynamics.
---

# MARSLib Control Theory Skill

You are a controls engineer for Team MARS 2614. When implementing motor control or tuning mechanisms:

## 1. Architecture

MARSLib's control hierarchy follows the **Feedforward > Feedback** principle:

| Layer | Runs On | Purpose |
|---|---|---|
| Feedforward (kS, kG, kV, kA) | roboRIO (subsystem) | Predicts voltage from physics model |
| PID (kP, kI, kD) | Motor controller (1kHz) | Corrects error from FF prediction |
| SlewRateLimiter | roboRIO (teleop) | Constrains driver input acceleration |
| Motion Profile | Motor controller (TalonFX) | Generates smooth position/velocity trajectories |

### Feedforward Classes
- `ElevatorFeedforward` — For linear mechanisms with gravity (`kS + kG + kV·v + kA·a`)
- `ArmFeedforward` — For rotational mechanisms with gravity (`kS + kG·cos(θ) + kV·v + kA·a`)
- `SimpleMotorFeedforward` — For flywheels/intakes without gravity (`kS + kV·v + kA·a`)

All feedforward gains are stored as `LoggedTunableNumber` instances so they can be tuned live from AdvantageScope without redeploying code.

## 2. Key Rules

### Rule A: Tune Feedforward Before PID
You MUST characterize the mechanism with SysId and set `kS`, `kG`, `kV` before touching `kP`. A well-tuned FF should get the mechanism to ~90% accuracy. PID only corrects the remaining error. If you start with PID, the system will oscillate or overshoot.

### Rule B: Discretize Swerve ChassisSpeeds
Never pass raw `ChassisSpeeds` directly to swerve modules. Always call `ChassisSpeeds.discretize(speeds, dt)` to compensate for coordinate skew during simultaneous translation and rotation within a 20ms loop tick. Without this, the robot curves instead of driving straight while spinning.

### Rule C: Clamp Driver Inputs with SlewRateLimiter
Wrap all teleop joystick inputs through `SlewRateLimiter` before passing to `SwerveDrive.drive()`. Without rate limiting, full-stick acceleration causes wheel slip, destroying odometry accuracy. Typical limits: 3.0 m/s² translation, 6.0 rad/s² rotation.

### Rule D: Use LoggedTunableNumber for All Gains
Never hardcode `kP = 5.0` in the constructor. Use `new LoggedTunableNumber("Elevator/kP", 5.0)`. This allows real-time tuning via AdvantageScope and ensures gains are logged for post-match analysis.

## 3. Adding Control Loops to New Mechanisms

1. **Run SysId** using the subsystem's built-in `sysIdQuasistatic()` and `sysIdDynamic()` commands.
2. **Extract gains** from the SysId analysis tool (kS, kG, kV, kA).
3. **Create `LoggedTunableNumber`** instances in the subsystem for each gain.
4. **Instantiate the correct `Feedforward` class** based on mechanism type.
5. **Set PID on the motor controller** via the IO layer's configuration method.
6. **Apply FF as voltage offset** in the subsystem's `setTargetPosition()` or `setTargetVelocity()`.

## 4. Constants
All control gains live in the mechanism's Constants inner class:
- `Constants.ElevatorConstants.kS/kG/kV/kA` — Elevator feedforward defaults
- `Constants.ArmConstants.kS/kG/kV/kA` — Arm feedforward defaults
- `Constants.DriveConstants.SLEW_RATE_TRANSLATION/ROTATION` — Driver input rate limits

## 5. Telemetry
- `{Mechanism}/kS`, `kG`, `kV`, `kA` — Current feedforward gains
- `{Mechanism}/TargetPosition` — Commanded setpoint
- `{Mechanism}/Position` — Actual position
- `{Mechanism}/AppliedVolts` — Total voltage (FF + PID output)
- `{Mechanism}/PositionError` — Setpoint minus actual
- `Swerve/DesiredSpeeds` — Pre-discretize commanded speeds
- `Swerve/DiscretizedSpeeds` — Post-discretize corrected speeds
