---
name: marslib-autonomous
description: Helps build autonomous code, PathPlanner macros, and GhostManager teleop replay commands in MARSLib.
---

# MARSLib Autonomous Skill

You are an expert FRC Software Developer. When configuring autonomous layouts, creating Paths, or managing the `GhostManager` for Team MARS 2614:

## 1. GhostManager (Teleop Replay)
- **Serialization Safety:** `GhostManager` relies on a thread-safe `ConcurrentLinkedQueue` daemon to ingest driver joystick inputs without disrupting the main loop. When analyzing macros, ensure file I/O operations are off-threaded.
- **Replay Injection:** During autonomous replays, push `CommandXboxController` overrides directly into the virtual memory buffers to perfectly spoof physical driver actuation to the core `RobotContainer` binds.

## 2. PathPlanner AutoBuilder Configuration
- Configure `AutoBuilder` using the ultra-high frequency `SwerveDrive` 250Hz Odometry Thread. Do not use standard 50Hz odometry callbacks for pathing.
- Handle multi-JVM configuration crashes natively in the Subsystem constructor by swallowing `already been configured` exceptions resulting from JUnit testing permutations.
- Inject intelligent event markers (like `ShootOnTheMoveCommand`) via NamedCommands.

## 3. MARSAlignment constraints
- When dynamically generating aiming paths or on-the-fly alignments (like aligning to the Subwoofer or Reef), use `MARSAlignmentCommand`.
- Explicitly enforce tolerance limits using WPILib's `ProfiledPIDController` for translation and `PIDController` for rotation (wrapped for continuous input `[-PI, PI]`).

## 4. Choreo Trajectory Deployment
When deploying pre-rendered Choreo trajectories:
- Use `MARSAuto.runChoreoTrajectory(name)` for simple playback from the `deploy/choreo` directory.
- Use `MARSAuto.pathfindThenRunChoreoTrajectory(name)` when the robot may not start exactly at the trajectory origin — it will dynamically pathfind to the start first.
- Use `MARSAuto.pathfindObstacleAvoidance(pose)` for dynamic teleop routing using PathPlanner's A* solver.
- All constraint parameters (`MAX_VELOCITY_MPS`, `MAX_ACCELERATION_MPS2`, etc.) are sourced from `Constants.AutoConstants`.
