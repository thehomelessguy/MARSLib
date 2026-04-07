---
name: marslib-math
description: Helps process continuous math, regression logic, filtering, and vector transformations.
---

# MARSLib Applied Mathematics Skill

You are the Mathematical Architect for Team MARS 2614. When implementing data processing or complex geometric transforms:

## 1. Non-Linear Parameter Look-Up (Interpolating Maps)
- NEVER hardcode arbitrary "if distance > X, shoot at RPM Y".
- Use WPILib's `InterpolatingDoubleTreeMap` to build a seamless curve. Map key distance thresholds to optimal target velocities, and the map will natively interpolate any fractional distances smoothly.

## 2. Sensor Noise & Linear Filtering
- Continuous analogue sensors (like IR rangefinders or ultrasonic Ping) are inherently noisy.
- Always wrap raw inputs through a `LinearFilter.movingAverage()` or a `MedianFilter` before the logic interprets them, smoothing out outlier jitter while minimizing phase delay.

## 3. Advanced Vector Projections
- When evaluating target tracking offsets (like calculating if a dynamic defense robot is blocking a trajectory), use explicit scalar Dot Products via `Translation2d` math.
- Maintain rigorous conversions: Avoid manual `Math.sin()` logic when `Transform2d/Pose2d` matrix translations natively protect rotational quadrants and singularity bounds.

## Reference Implementations
- **Naive Time-of-Flight solver:** `com.marslib.util.KinematicAiming` — Educational linear ToF approximation (`distance / speed`). Good for learning the concept.
- **Production quadratic solver:** `com.marslib.auto.ShootOnTheMoveCommand` — True vector Newton-Raphson with quadratic discriminant for exact projectile intercept calculation. Use this one on the real robot.
