---
name: marslib
description: Helps write FRC robot code using the MARSLib Advanced Simulation and Abstraction framework. Use when creating new subsystems, writing hardware IO layers, configuring Dyn4j simulation physics, or setting up fault management.
---

# MARSLib Framework Skill

You are an expert FRC Software Engineer for Team MARS 2614. When asked to create new robot subsystems, commands, or mechanisms for a MARSLib-based project, adhere strictly to the following architectural guidelines.

## 1. Scaffolding Subsystems (AdvantageKit Rule)
Every subsystem MUST have its hardware interaction abstracted behind an IO interface. You must NEVER instantiate hardware directly in the subsystem class. You must generate exactly four files:
1. **`[Name]IO.java`**: Interface with an `@AutoLog` inner class `[Name]IOInputs`. All hardware reading methods update this inputs object.
2. **`[Name]IOSim.java`**: Physics sim using `MARSPhysicsWorld`. Use `dyn4j` to simulate mechanisms, calculating current draw and physics odometry dynamically.
3. **`[Name]IOTalonFX.java`**: The Hardware class. Directly interfaces with CAN motors and sensors.
4. **`[Name].java`**: The actual subsystem that accepts `[Name]IO` via dependency injection on the constructor.

Periodically perform `io.updateInputs(inputs)` and call `Logger.processInputs("[Name]", inputs)` in the `periodic()` loop.
Do NOT use `SmartDashboard`; use `LoggedTunableNumber`.

## 2. Superstructure & Collision Prevention
When writing a command that requires multiple mechanisms to move simultaneously, do NOT use WPILib's `ParallelCommandGroup`.
Instead, queue a request in `MARSSuperstructure.java`. The Superstructure must evaluate physical constraints natively (e.g., "Elevator must be above 20 inches before Arm can pivot") via dynamic conditionals (like `Commands.either()`) before dispatching the states to the respective subsystems dynamically.

## 3. Motor Configuration Rule (Phoenix 6)
All CTRE hardware uses the **Phoenix 6 API**. Do not use Phoenix 5 (`TalonFXControlMode` or `WPI_TalonFX`).
- Always use `TalonFXConfiguration` objects to apply settings.
- You must enforce `StatorCurrentLimit` and `SupplyCurrentLimit` on all motors to prevent roboRIO brownouts.
- Use `BaseStatusSignal.setUpdateFrequency()` to lower the CAN bus utilization of non-essential telemetry (like motor temperature).
- Use `MARSPowerManager` to dynamically step down and track limits if the voltage drops to critical bounds.

## 4. Fault Management
Always pipe hardware failures or timeout detections through `MARSFaultManager`.
- If a motor initialization fails or a CAN frame continually drops, use `MARSFaultManager.registerCriticalFault()`.
- Alerts should be posted using AdvantageScope's Alert class (e.g. `new Alert("Elevator Disconnected", AlertType.CRITICAL).set(true);`).

## 5. Simulation Integration (Dyn4j)
When writing `[Name]IOSim.java` layers:
- Register the `Body` logic to the centralized `MARSPhysicsWorld`.
- Provide basic internal profile controllers to roughly mimic 1kHz motor controllers (like TalonFX Motion Magic).
- Always aggregate the simulated current draw up to `MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(currentDrawAmps)`.
- Use the overarching dyn4j physics constraint parameters (e.g., Stribeck/Coulomb friction mappings) to drive true, uncheated wheel slip.

## 6. Digital Twin & Stochastic Fidelity
MARSLib maintains "Einstein-ready" physical simulation parity through configurable noise models located in `Constants.SimulationConstants`:
- **Sensor Noise**: Never return mathematically perfect values. Wrap Gyro yaws in Gaussian white noise and static drift vectors.
- **Odometry & Drift**: Rely on native physical acceleration limits; `dyn4j` naturally slips wheels under heavy tractive load, inherently de-syncing odometry unless mitigated with simulated vision.
- **Vision Occlusion**: Do not assume 100% camera lock. Implement stochastic probabilistic frame-dropping to mirror real-world obstructions.
- **Hardware Throttling**: Use $I^2R$ electrical modeling to track stator heat buildup over a match. Actively clamp and fade maximum output motor torque matrices if temperatures exceed safe margins (90°C).
- **Spatial Awareness**: Rely on dynamic raycasting (e.g. `LidarIOSim`) for physical collision-detection rather than cheating and reading the global `Odometry` state. Push collision depth matrices natively to `Translation3d[]` clouds in AdvantageKit.

Always build modular, fail-safe code designed strictly for deterministic log-replay analysis.

## 7. Integrated Simulation Testing (JUnit)
When writing unit tests for MARSLib commands or subsystems, do NOT use `Mockito` to mock the hardware layer. Instead, use WPILib's native `CommandScheduler` mapping to `SwerveModuleIOSim` and `dyn4j` to achieve physical environment parity.
- **Physics Resetting**: The `MARSPhysicsWorld` is a static singleton. In a multi-test JUnit suite, failing to clear it causes chassis instances to stack up at `(0,0)`, locking the engine due to infinite friction bounds. ALWAYS manually hook `com.marslib.simulation.MARSPhysicsWorld.resetInstance();` in your `@BeforeEach` `setUp()` block alongside `CommandScheduler.getInstance().cancelAll()`.
- **DriverStation Timeouts**: WPILib implements intrinsic disabling loops if `DriverStationSim` heartbeat timeouts drop. Inside any 150-tick integration loop where `SimHooks.stepTiming(LOOP_PERIOD_SECS)` and `CommandScheduler.getInstance().run()` are stepped, you MUST continuously call `DriverStationSim.notifyNewData();` to ensure virtual target commands don't abruptly terminate.
- **Config Crash Bypassing**: Ensure elements like PathPlanner's `AutoBuilder.configure()` gracefully catch and ignore static-linked crashes (e.g. `already been configured`) which natively persist across JUnit runner instances.

## 8. Creating New Game Simulations
When configuring MARSLib for a new FRC season (e.g., the 2026 REBUILT game):
- **Collision Boundaries:** Never assume a blank field. Inject rigid body obstacles directly into `MARSPhysicsWorld.java`. Use methods like `MARSPhysicsWorld.buildStaticHexagon()` or standard `dyn4j` geometry functions to map solid CAD structures.
- **Material Properties:** Configure `dyn4j` materials accurately. Field perimeters should have high friction (`0.8`) and low restitution/bounciness (`0.1`) to prevent physics clipping when swerve drives ram into them.
- **Field Constants:** Store all obstacle radii, coordinates, and origin mappings in `Constants.FieldConstants` so they remain globally accessible across standard robot logic and the physics engine.
- **Dynamic Interaction:** If game pieces are ingestible, define mathematically bound ranges (e.g., `checkIntake(pose, radius)`) and log their 3D states sequentially to `AdvantageScope` so students visually confirm interactions.

## 9. Game Piece Physical Constants
When configuring game piece properties for a new season, update `Constants.FieldConstants`:
- **`GAME_PIECE_RADIUS_METERS`** — The physical radius (e.g., `0.0635` for 5" diameter REBUILT Fuel).
- **`GAME_PIECE_MASS_KG`** — The mass (e.g., `0.05` for foam balls).
- **`INTAKE_COLLECTION_RADIUS_METERS`** — The proximity threshold for `checkIntake()`.

For 2026 REBUILT, the field stages **168 total Fuel**: 24 per alliance Depot (4×6 grid) and 120 in the Neutral Zone midline (5×24 grid with row-alternating jitter). See `MARSPhysicsWorld.spawnInitialGamePieces()` for the canonical implementation. For deeper simulation guidance, refer to the `marslib-simulation` skill.
