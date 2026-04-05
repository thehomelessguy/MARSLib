---
name: marslib
description: Helps write FRC robot code using the MARSLib Advanced Simulation and Abstraction framework. Use when creating new subsystems, writing hardware IO layers, configuring Dyn4j simulation physics, or setting up fault management.
---

# MARSLib Framework Skill

When asked to create new robot subsystems, commands, or mechanisms for a MARSLib-based project, adhere strictly to the following architectural guidelines. 

## 1. IO Layer Abstraction (AdvantageKit)
Every subsystem MUST have its hardware interaction abstracted behind an IO interface. 
Never put hardware objects (TalonFX, CANSparkMax, Solenoids) directly inside a Subsystem class.
- **`[Name]IO.java`**: The interface file. It must contain the `@AutoLog` inner class `[Name]IOInputs`. All hardware reading methods update this inputs object.
- **`[Name]IOReal.java`**: The real hardware implementation. Directly interfaces with CAN motors and sensors.
- **`[Name]IOSim.java`**: The physics simulation implementation. Use `MARSPhysicsWorld` and `dyn4j` to simulate mechanisms, calculating current draw and physics odometry dynamically.

## 2. Setting Up Subsystems
When building the `{Name}Subsystem.java`:
- It must inherit from `SubsystemBase`.
- Provide the `[Name]IO` interface via constructor injection. No real hardware initialization in the constructor.
- Periodically perform `io.updateInputs(inputs)` and call `Logger.processInputs("[Name]", inputs)` in the `periodic()` loop.
- Use `MARSPowerManager` to monitor battery voltage and request dynamic load shedding (active current limit clamping) if the voltage approaches critical brownout levels.

## 3. Fault Management
Always pipe hardware failures or timeout detections through `MARSFaultManager`.
- If a motor initialization fails or a CAN frame continually drops, use `MARSFaultManager.registerCriticalFault()`.
- Alerts should be posted using AdvantageScope's Alert class (e.g. `new Alert("Elevator Disconnected", AlertType.CRITICAL).set(true);`).

## 4. Simulation Integration (Dyn4j)
When writing `[Name]IOSim.java` layers:
- Register the `Body` logic to the centralized `MARSPhysicsWorld`. 
- Provide basic internal profile controllers to roughly mimic 1kHz motor controllers (like TalonFX Motion Magic).
- Always aggregate the simulated current draw up to `MARSPhysicsWorld.getInstance().addFrameCurrentDrawAmps(currentDrawAmps)`.

Always build modular, fail-safe code designed strictly for deterministic log-replay analysis.
