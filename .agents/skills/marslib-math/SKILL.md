---
name: marslib-math
description: Helps process continuous math, regression logic, filtering, and vector transformations. Use when implementing interpolation maps, sensor filtering, projectile calculations, or geometric transforms.
---

# MARSLib Applied Mathematics Skill

You are the mathematical architect for Team MARS 2614. When implementing data processing or geometric calculations:

## 1. Architecture

Math utilities span `com.marslib.util` and `com.marslib.auto`:

| Class | Purpose |
|---|---|
| `KinematicAiming` | Linear time-of-flight solver (`distance / speed`). Educational, not production-ready. |
| `ShootOnTheMoveCommand` | Newton-Raphson quadratic projectile intercept solver. Use this for competition. |
| `LoggedTunableNumber` | AdvantageKit-compatible tunable double with automatic logging and change detection. |

### WPILib Math Utilities Used
MARSLib relies heavily on WPILib's math library rather than reimplementing:
- `InterpolatingDoubleTreeMap` — Non-linear parameter lookup with smooth interpolation
- `LinearFilter.movingAverage()` — Noise reduction for analog sensors
- `MedianFilter` — Outlier-robust filtering
- `Translation2d`, `Rotation2d`, `Pose2d`, `Transform2d` — Geometric transforms with quadrant-safe trigonometry

## 2. Key Rules

### Rule A: Never Hardcode Lookup Tables
Do NOT write `if (distance > 3.0) rpm = 4000; else if (distance > 2.0) rpm = 3500`. Use `InterpolatingDoubleTreeMap` instead — it provides smooth interpolation between data points and handles edge cases automatically.

### Rule B: Always Filter Analog Sensors
Raw analog inputs (IR rangefinders, ultrasonic) are noisy. Wrap them in `LinearFilter.movingAverage(5)` or `MedianFilter(5)` before using the value in control logic. This prevents jittery mechanism behavior.

### Rule C: Use WPILib Geometry, Not Manual Trig
Never write `Math.atan2(dy, dx)` manually when `Translation2d` or `Rotation2d` provides the same operation with proper quadrant handling and singularity protection. Use `Pose2d.relativeTo()` and `Transform2d` for coordinate frame conversions.

### Rule D: LoggedTunableNumber for Runtime Constants
Any constant that might need tuning (PID gains, distances, speeds) MUST use `LoggedTunableNumber`. This enables real-time tuning from AdvantageScope and ensures the value is logged for post-match analysis:
```java
private final LoggedTunableNumber shotSpeed = new LoggedTunableNumber("Shooter/ShotSpeed", 15.0);
```

## 3. Adding New Math Utilities

1. Place pure math utilities in `com.marslib.util`.
2. Place command-level math (aiming, following) in `com.marslib.auto`.
3. Write a JUnit test that verifies the math against known analytic solutions.
4. Use `LoggedTunableNumber` for any parameters that may need tuning.
5. Document the mathematical model in the class Javadoc — students need to understand the physics.

## 4. Reference Implementations
- **Linear ToF:** `KinematicAiming.calculateLeadAngle()` — Simple `distance / speed` with linear lead. Good for learning.
- **Quadratic ToF:** `ShootOnTheMoveCommand` — True projectile intercept with Newton-Raphson iteration. Handles moving targets, gravity, and spin. Use this on the real robot.

## 5. Telemetry
- `KinematicAiming/LeadAngle` — Calculated lead angle in radians
- `KinematicAiming/TimeOfFlight` — Estimated projectile flight time
- `ShootOnTheMove/ConvergedSolution` — Boolean: Newton-Raphson converged
- `ShootOnTheMove/IterationCount` — Solver iterations before convergence
- `{Name}/TunableValue` — Current value of any `LoggedTunableNumber`
