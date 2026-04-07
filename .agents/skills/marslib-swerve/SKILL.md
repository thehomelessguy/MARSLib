---
name: marslib-swerve
description: Helps configure and extend the MARSLib swerve drivetrain subsystem. Use when modifying SwerveDrive, adding odometry sources, tuning PathPlanner, or configuring the PhoenixOdometryThread.
---

# MARSLib Swerve Drivetrain Skill

You are a drivetrain engineer for Team MARS 2614. When modifying the swerve subsystem or adding new odometry sources:

## 1. Architecture

The swerve drivetrain is split across 9 files in `com.marslib.swerve`:

| File | Purpose |
|---|---|
| `SwerveDrive.java` | Top-level subsystem — kinematics, pose estimation, PathPlanner, power shedding |
| `SwerveModule.java` | Per-module wrapper — processes IO inputs, runs SysId, logs per-module telemetry |
| `SwerveModuleIO.java` | IO interface with `@AutoLog` inputs (drive/turn position, velocity, current) |
| `SwerveModuleIOTalonFX.java` | Real hardware — Phoenix 6 TalonFX + CANcoder |
| `SwerveModuleIOSim.java` | Physics sim — dyn4j-backed wheel slip with Stribeck friction |
| `GyroIO.java` | Gyro interface with `@AutoLog` inputs (yaw, pitch, roll, rates) |
| `GyroIOPigeon2.java` | Real hardware — CTRE Pigeon2 IMU |
| `GyroIOSim.java` | Physics sim — derives yaw from chassis speeds + Gaussian noise |
| `PhoenixOdometryThread.java` | Background 250Hz thread draining TalonFX signal queues |

### Dependency Injection
```java
// In RobotContainer:
SwerveModule[] modules = new SwerveModule[] {
    new SwerveModule(0, new SwerveModuleIOTalonFX(0)),  // or IOSim
    new SwerveModule(1, new SwerveModuleIOTalonFX(1)),
    new SwerveModule(2, new SwerveModuleIOTalonFX(2)),
    new SwerveModule(3, new SwerveModuleIOTalonFX(3))
};
SwerveDrive drive = new SwerveDrive(modules, new GyroIOPigeon2(), powerManager);
drive.configurePathPlanner(); // MUST be called separately — not in constructor
```

## 2. Key Rules

### Rule A: PathPlanner is Decoupled
`AutoBuilder.configure()` is NOT called in the `SwerveDrive` constructor. It lives in the public method `configurePathPlanner()` which must be called by `RobotContainer` after construction. This prevents the AutoBuilder static singleton from crashing test classes that construct SwerveDrive.

### Rule B: Module Indices Are Sacred
Module indices 0-3 map to FL, FR, BL, BR. The order is defined in `SwerveConstants.MODULE_LOCATIONS`. Mismatching indices causes the kinematics to invert, producing diagonal driving. Never reorder without updating both arrays.

### Rule C: Odometry Thread Must Be Drained
`PhoenixOdometryThread` accumulates position samples at 250Hz into lock-free queues. The `SwerveDrive.periodic()` method drains these queues and feeds them to the `SwerveDrivePoseEstimator`. If `periodic()` is not called (e.g., subsystem not registered), the queues grow unbounded.

### Rule D: Power Shedding is Continuous
`SwerveDrive.periodic()` polls `MARSPowerManager` every tick. If voltage drops below `CRITICAL_VOLTAGE`, stator current limits are dynamically clamped via CAN to prevent brownout. This is non-negotiable — never bypass it.

## 3. Adding Odometry Sources
To add a new odometry source (e.g., VIO SLAM from `MARSVision`):
1. Call `swerveDrive.addVisionMeasurement(pose, timestamp, stdDevs)` from the vision subsystem's periodic.
2. Tune the standard deviations in `Constants.VisionConstants` — higher std devs = less trust in vision.
3. See the `marslib-vision` skill for the vision pipeline details.

## 4. Constants
All drivetrain constants live in `frc.robot.SwerveConstants`:
- `MODULE_LOCATIONS` — Translation2d array defining module positions relative to robot center
- `DRIVE_GEAR_RATIO`, `TURN_GEAR_RATIO` — Gear reductions
- `WHEEL_RADIUS_METERS` — Wheel radius for distance calculation
- `MAX_LINEAR_SPEED_MPS` — Maximum achievable speed
- `ROBOT_MASS_KG`, `ROBOT_MOI_KG_M2` — For PathPlanner config

## 5. Telemetry
- `Swerve/Pose` — Estimated Pose2d
- `Swerve/ModuleStates` — Current SwerveModuleState[]
- `Swerve/DesiredStates` — Commanded SwerveModuleState[]
- `Swerve/ChassisSpeeds` — Current ChassisSpeeds
- `Swerve/OdometryPose` — Raw odometry (no vision) for comparison
