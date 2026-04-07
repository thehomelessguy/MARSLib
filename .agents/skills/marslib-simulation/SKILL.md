---
name: marslib-simulation
description: Helps configure and extend the MARSLib dyn4j physics simulation engine. Use when spawning game pieces, registering mechanism bodies, adding field obstacles, or debugging physics performance.
---

# MARSLib Simulation & Physics Skill

You are a Physics Simulation Engineer for Team MARS 2614. When modifying or extending the `dyn4j`-based physics environment:

## 1. Architecture Overview
MARSLib's simulation is driven by four core classes:
- **`MARSPhysicsWorld`** — Singleton orchestrator. Owns the `dyn4j` `World`, manages field boundaries, game pieces, mechanism bodies, and the 50Hz update loop.
- **`SwerveChassisPhysics`** — Translates WPILib `SwerveModuleState` wheel commands into `dyn4j` force vectors on the chassis `Body`. Handles wheel slip, friction, and angular momentum.
- **`GamePieceSim`** — Represents a single dynamic game piece (e.g., Fuel). Automatically registers itself with the physics world on construction.
- **`LidarIOSim`** — Raycasts 360° from the robot chassis to detect walls and game pieces, logging the result as a `Translation3d[]` point cloud.

## 2. Spawning Game Pieces
All initial game piece staging is handled in `MARSPhysicsWorld.spawnInitialGamePieces()`:
- Physical constants (`GAME_PIECE_RADIUS_METERS`, `GAME_PIECE_MASS_KG`) live in `Constants.FieldConstants`.
- For the 2026 REBUILT game, **168 total Fuel** are spawned: 24 per depot (Blue/Red) and 120 in the Neutral Zone midline.
- Use grid loops with `spacing = radius * 2.5` to prevent overlapping bodies.
- Add randomized jitter (`(row % 2 == 0) ? 0.05 : -0.05`) to prevent rigid interlocking at spawn.

## 3. Registering Mechanism Bodies
When writing `[Name]IOSim.java` classes that participate in physics:
- Create `dyn4j` `Body` objects for the mechanism (e.g., anchor + carriage for a `LinearMechanismIOSim`).
- Register them via `MARSPhysicsWorld.getInstance().getWorld().addBody(body)` for collision participation.
- Register named references via `MARSPhysicsWorld.getInstance().registerMechanismBody(name, body)` for cross-subsystem lookups.
- Always aggregate simulated current draw: `MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(amps)`.

## 4. Field Obstacles & Boundaries
- Field perimeter walls are injected as `MassType.INFINITE` static bodies in `MARSPhysicsWorld`.
- Use high friction (`0.8`) and low restitution (`0.1`) for walls to prevent physics clipping.
- For game-specific structures (e.g., Hubs, Depots), use `Geometry.createRectangle()` or `Geometry.createCircle()` for collision shapes.
- Store all obstacle coordinates in `Constants.FieldConstants`.

## 5. Intake & Scoring Interaction
Game piece collection is managed by `MARSPhysicsWorld.checkIntake(Pose2d robotPose, double radius)`:
- Iterates all active `GamePieceSim` bodies and checks if any are within the collection radius.
- The closest piece is removed from the `dyn4j` world and returns `true`.
- The `MARSSuperstructure` calls this every loop when in `INTAKE_FLOOR` state.
- Scoring resets `hasPiece = false` and optionally re-spawns the piece at a scoring location.

## 6. Performance Considerations
- **168 dynamic bodies** at 50Hz is the current upper bound for smooth simulation.
- If frame times exceed 20ms, consider:
  - Reducing `GamePieceSim` linear/angular damping to settle faster.
  - Implementing spatial partitioning or "sleep" thresholds on stationary pieces.
  - Reducing the Neutral Zone count to ~80 pieces.
- Always monitor via `Logger.recordOutput("PhysicsSim/StepTimeMs", ...)`.

## 7. Voltage Sag Simulation
`MARSPhysicsWorld` tracks total frame current draw from all mechanisms. The aggregated draw is fed into `PowerIOSim` to dynamically simulate battery voltage sag:
- At high current (>120A), voltage drops below `Constants.PowerConstants.WARNING_VOLTAGE`.
- This triggers `MARSPowerManager` alerts and `LEDManager` load-shedding colors.
- Swerve drive may receive reduced voltage headroom, simulating real brownout behavior.
