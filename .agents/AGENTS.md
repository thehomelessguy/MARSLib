# MARSLib Project Rules

## 1. Zero Direct Telemetry API Access
Agents must NEVER call the direct WPILib or FTC standard telemetry apis (e.g. `SmartDashboard.putNumber` or `telemetry.addData()`).
All logging **must** filter through strict AdvantageKit IO layer objects. Hardware data is populated inside `@AutoLog` `[Name]Inputs` objects and updated periodically.

## 2. Universal Hardware Abstraction
Never instantiate raw motor controllers (`TalonFX`, `DcMotorEx`, `CANSparkMax`) directly in Subsystem files.
1. Create a `[Name]IO` interface.
2. Create actual hardware bridges in `[Name]IOReal`.
3. Require `[Name]IO` in the subsystem constructor via dependency injection.
This is non-negotiable for enabling identical logic execution in test-driven simulation modules.

## 3. High-Fidelity Desktop Simulation
Whenever applicable, ensure complex geometries are fully simulated inside our `DesktopSimLauncher`.
- **dyn4j**: Use `dyn4j` to define strictly static bounding walls, active physical shapes, and precise Center-Of-Mass derivations.
- **Odometry Mismatch**: Simulate wheel scrub natively. `ArrayOdometryIOSim` / `SwerveModuleIOSim` logic should use integrated physics velocities rather than flawlessly obeying inverse kinematics equations, generating "realistic" tracking drift natively in simulation.
