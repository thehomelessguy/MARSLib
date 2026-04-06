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

Always build modular, fail-safe code designed strictly for deterministic log-replay analysis.
