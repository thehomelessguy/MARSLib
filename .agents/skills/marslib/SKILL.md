---
name: marslib
description: Helps write FRC robot code using the MARSLib Advanced Simulation and Abstraction framework. Use when creating new subsystems, writing hardware IO layers, configuring Dyn4j simulation physics, or setting up fault management.
---

# MARSLib Framework Skill

You are an expert FRC Software Engineer for Team MARS 2614. This is the **root skill** — it defines the core architectural rules that apply to ALL MARSLib code. For domain-specific guidance, see the dedicated skills listed below.

## 1. IO Abstraction (AdvantageKit Rule)

Every subsystem MUST have its hardware interaction abstracted behind an IO interface. You must NEVER instantiate hardware directly in the subsystem class. Generate exactly four files:
1. **`[Name]IO.java`**: Interface with an `@AutoLog` inner class `[Name]IOInputs`.
2. **`[Name]IOSim.java`**: Physics sim using `MARSPhysicsWorld` and dyn4j.
3. **`[Name]IOTalonFX.java`**: Real hardware via Phoenix 6 API.
4. **`[Name].java`**: Subsystem that accepts `[Name]IO` via dependency injection.

In `periodic()`: call `io.updateInputs(inputs)` then `Logger.processInputs("[Name]", inputs)`.

> For detailed mechanism patterns, see `marslib-mechanisms`.

## 2. Core Constraints

These rules are **non-negotiable** across all MARSLib code:

| Rule | Details | Skill |
|---|---|---|
| No SmartDashboard | Use `Logger.recordOutput()` and `LoggedTunableNumber` only | `marslib-telemetry` |
| No Mockito in tests | Use `*IOSim` with dyn4j physics instead | `marslib-testing` |
| Phoenix 6 only | No Phoenix 5 APIs (`WPI_TalonFX`, `TalonFXControlMode`) | `marslib-power` |
| Dual current limits | Always set both Stator and Supply limits on TalonFX | `marslib-power` |
| CAN bus < 80% | Drop non-critical signal frequencies to 4-10Hz | `marslib-power` |
| Collision routing | Multi-mechanism moves go through `MARSSuperstructure` | `marslib-superstructure` |
| Faults through manager | All hardware errors go via `MARSFaultManager` | `marslib-diagnostics` |

## 3. Domain Skills Index

| Domain | Skill | Covers |
|---|---|---|
| Drivetrain | `marslib-swerve` | SwerveDrive, odometry, PathPlanner, modules |
| Mechanisms | `marslib-mechanisms` | Elevator, arm, intake, shooter IO patterns |
| Superstructure | `marslib-superstructure` | Collision safety, state machine coordination |
| State Machines | `marslib-statemachine` | Generic FSM framework (`MARSStateMachine<S>`) |
| Autonomous | `marslib-autonomous` | PathPlanner, GhostManager, Choreo, alignment |
| Ghost Replay | `marslib-ghost` | GhostManager recording, playback, macro files |
| Shot Setup | `marslib-shotsetup` | EliteShooterMath SOTM solver, time-of-flight |
| Vision | `marslib-vision` | AprilTag fusion, VIO SLAM, camera simulation |
| Simulation | `marslib-simulation` | dyn4j physics, field boundaries, game pieces |
| Controls | `marslib-control-theory` | PID, feedforward, SysId, slew rate limiting |
| Power | `marslib-power` | Current limits, CAN bus, brownout protection |
| Telemetry | `marslib-telemetry` | AdvantageKit logging, replay determinism |
| Diagnostics | `marslib-diagnostics` | Faults, alerts, pre-match system checks |
| Operator | `marslib-operator` | Controller bindings, haptics, LEDs, dashboard |
| Math | `marslib-math` | Interpolation, filtering, vector transforms |
| Network | `marslib-network` | NT4, coprocessor streams, bandwidth |
| Testing | `marslib-testing` | JUnit 5, singleton resets, physics loops |
| Elite Mining | `marslib-elite-mining` | Cross-team code mining & architectural analysis |
| CI/CD | `marslib-ci` | Gradle, Spotless, GitHub Actions |
| Skill Authoring | `marslib-skill-authoring` | How to create new skills |

## 4. Telemetry
See each domain skill's **Telemetry** section for exhaustive log key listings. The root skill does not emit its own telemetry.
