---
name: marslib-simulation
description: Helps configure and extend the MARSLib dyn4j physics simulation engine. Use when spawning game pieces, registering mechanism bodies, adding field obstacles, or debugging physics performance.
---

# MARSLib Simulation & Physics Skill

You are a physics simulation engineer for Team MARS 2614. When modifying or extending the dyn4j environment:

## 1. Architecture

MARSLib's simulation is driven by four core classes in `com.marslib.simulation`:

| Class | Purpose |
|---|---|
| `MARSPhysicsWorld` | Singleton orchestrator. Owns the dyn4j `World`, manages field boundaries, game pieces, mechanism bodies, and the 50Hz update loop. |
| `SwerveChassisPhysics` | Translates WPILib `SwerveModuleState` wheel commands into dyn4j force vectors. Handles wheel slip, friction, and angular momentum. |
| `GamePieceSim` | Represents a single dynamic game piece. Auto-registers with the physics world. |
| `LidarIOSim` | 360° raycasting from the chassis for point cloud generation. |

### Singleton Pattern
`MARSPhysicsWorld` is a **static singleton**. This is necessary because multiple IO layers (elevator, arm, swerve) must share the same physics world. However, this creates a test isolation problem — see Rule C.

## 2. Key Rules

### Rule A: Register All Mechanism Bodies
Every `*IOSim` that participates in physics MUST register its dyn4j `Body` objects:
```java
MARSPhysicsWorld.getInstance().getWorld().addBody(body);
MARSPhysicsWorld.getInstance().registerMechanismBody("Elevator", body);
```
Unregistered bodies won't collide with anything and won't be stepped by the physics engine.

### Rule B: Aggregate Current Draw
Every `*IOSim` MUST report its simulated current draw every frame:
```java
MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(currentAmps);
```
This feeds the `PowerIOSim` voltage sag model. Without it, the sim never brownouts and hides real power issues.

### Rule C: Reset Instance in Every Test
The singleton MUST be reset before every test to prevent physics bodies from stacking:
```java
@BeforeEach
public void setUp() {
    MARSPhysicsWorld.resetInstance();
}
```
Without this, chassis instances accumulate at (0,0), creating infinite friction and locking the physics engine.

### Rule D: Use Real Materials, Not Defaults
Field perimeter walls: friction `0.8`, restitution `0.1` (high grip, low bounce). Game pieces: friction `0.3`, restitution `0.5`. Default dyn4j values will cause pieces to clip through walls or robots to slide unrealistically.

## 3. Adding New Field Elements

1. Create static bodies in `MARSPhysicsWorld` using `Geometry.createRectangle()` or `createCircle()`.
2. Set `body.setMass(MassType.INFINITE)` for immovable obstacles.
3. Configure materials: high friction for walls, moderate for scoring targets.
4. Store coordinates in `Constants.FieldConstants`.
5. Verify collisions in a test by driving the chassis into the new obstacle.

## 4. Game Piece Configuration

All initial piece staging lives in `MARSPhysicsWorld.spawnInitialGamePieces()`:
- Physical constants in `Constants.FieldConstants`: `GAME_PIECE_RADIUS_METERS`, `GAME_PIECE_MASS_KG`
- For 2026 REBUILT: **168 Fuel** — 24/depot (Blue+Red) + 120 Neutral Zone midline
- Grid spacing: `radius * 2.5` to prevent overlap
- Row jitter: `±0.05m` alternating to prevent rigid interlocking

## 5. Performance Limits
- **168 dynamic bodies** at 50Hz is the current upper bound
- If frame times exceed 20ms: reduce damping, implement sleep thresholds, or reduce piece count
- Monitor via `Logger.recordOutput("PhysicsSim/StepTimeMs", ...)`

## 6. Telemetry
- `PhysicsSim/StepTimeMs` — Physics engine step duration
- `PhysicsSim/BodyCount` — Total active dyn4j bodies
- `PhysicsSim/TotalCurrentDraw` — Aggregate current from all mechanisms
- `PhysicsSim/GamePiecePoses` — Pose2d[] of all active game pieces (for AdvantageScope 2D field)
- `PhysicsSim/ChassisVelocity` — Physics-true chassis velocity (not odometry)
