---
name: marslib-shotsetup
description: Helps configure and tune the Shot-On-The-Move (SOTM) iterative solver for dynamic shooting. Use when adjusting shot maps, time-of-flight curves, or convergence parameters.
---

# MARSLib Shot Setup Skill

You are the ballistics engineer for Team MARS 2614. When tuning shooting parameters or extending the SOTM system:

## 1. Architecture

`ShotSetup` (`com.marslib.util`) is a 232-line utility that computes optimal shooter RPM and cowl angle for moving shots:

### Solver Pipeline
```
1. Measure distance to target
2. Predict robot pose at time-of-flight (using velocity + phase delay)
3. Interpolate RPM and cowl angle from distance→shot map
4. Iterate: recalculate distance from predicted pose, re-interpolate
5. Converge within epsilon or exceed max iterations
6. Cache result until distance changes by recomputeThreshold
```

### Key Parameters (set in RobotContainer)
| Parameter | Default | Purpose |
|---|---|---|
| `phaseDelay` | 0.1s | Lookahead for mechanism latency (cowl + flywheel spin-up) |
| `maxCowlPosition` | π/2 rad | Upper bound for cowl angle |
| `maxFlywheelSpeedRPM` | 6000 | Maximum flywheel speed |
| `recomputeThreshold` | 0.25m | Distance change required to trigger recalculation |
| `convergenceIters` | 5 | Maximum convergence iterations |
| `convergenceEpsilon` | 0.01m | Convergence tolerance |
| `minTof` / `maxTof` | 0.1–1.5s | Time-of-flight bounds |
| `shooterTransform` | (0,0) | Shooter position offset from robot center |
| `operatorForward` | 0° | Forward direction offset |

### Interpolation Maps
Two separate `InterpolatingDoubleTreeMap`-style maps are populated:
```java
// Distance → (RPM, CowlAngle) map
shotSetup.addShotMapEntry(1.0, 2000.0, 0.15);
shotSetup.addShotMapEntry(3.0, 3000.0, 0.35);
shotSetup.addShotMapEntry(5.0, 4000.0, 0.55);
shotSetup.addShotMapEntry(7.0, 4800.0, 0.75);
shotSetup.addShotMapEntry(10.0, 5500.0, 1.0);

// Distance → Time-of-Flight (seconds) map
shotSetup.addTofMapEntry(1.0, 0.15);
shotSetup.addTofMapEntry(3.0, 0.25);
shotSetup.addTofMapEntry(5.0, 0.40);
shotSetup.addTofMapEntry(7.0, 0.60);
shotSetup.addTofMapEntry(10.0, 0.90);
```

## 2. Key Rules

### Rule A: Static vs Moving Shots
- `getStaticShotInfo(distance)` — Returns RPM + cowl for a stationary robot. Used by `MARSSuperstructure`.
- `getMovingShotInfo(robotPose, robotVelocity, targetPose)` — Runs the iterative solver with lead compensation. Used by `ShootOnTheMoveCommand`.

### Rule B: Caching with Distance Hysteresis
The solver caches the last computed result and only recomputes when the distance changes by more than `recomputeThreshold` (default 0.25m). This prevents jitter in cowl angle and RPM.

### Rule C: Phase Delay Compensation
The solver predicts where the robot will be at `currentTime + timeOfFlight + phaseDelay`. This compensates for:
- Flywheel spin-up time
- Cowl positioning time
- Note: `phaseDelay` should be tuned empirically by measuring system latency

### Rule D: Angular Velocity Feedforward
The moving shot solver applies omega feedforward to the heading target, so the cowl angle accounts for the robot rotating during the shot's flight time.

## 3. Tuning Workflow
1. **Characterize stationary shots:** Map distance→(RPM, cowlAngle) on the real field with a tape measure
2. **Add entries:** Call `addShotMapEntry()` and `addTofMapEntry()` in `RobotContainer`
3. **Test in sim:** Run `simulateJava`, drive to different distances, verify `Superstructure/GoalCowlAngle` and shooter RPM match expectations
4. **Tune convergence:** If shots are inconsistent, increase `convergenceIters` or decrease `convergenceEpsilon`
5. **Tune phase delay:** If shots land behind/ahead of the target while moving, adjust `phaseDelay`

## 4. Telemetry
- `ShotSetup/SolverDistanceM` — Current distance to target
- `ShotSetup/SolverRPM` — Computed target RPM
- `ShotSetup/SolverCowlAngle` — Computed cowl angle (radians)
- `ShotSetup/SolverIterations` — Convergence iterations used
- `ShotSetup/CacheHit` — Whether the cached result was reused
