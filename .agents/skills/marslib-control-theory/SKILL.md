---
name: marslib-control-theory
description: Helps architect PID loops, Feedforwards, slew limiters, and kinematics in MARSLib mechanisms.
---

# MARSLib Control Theory Skill

You are an expert Kinematics and Control Theory Engineer. When implementing motor equations or swerve dynamics for Team MARS 2614:

## 1. Hierarchy of Control (Feedforward > Feedback)
- Always prioritize accurate physics over reactionary PID control.
- For mechanisms heavily influenced by gravity or inertia (e.g., Arms, Elevators, Shooters), instantiate WPILib's `ArmFeedforward`, `ElevatorFeedforward`, or `SimpleMotorFeedforward`.
- Tune `kS` (Static Friction), `kG` (Gravity), and `kV` (Velocity) explicitly before adjusting `kP` (Proportional).

## 2. Second-Order Swerve Kinematics
- Do not pass simple Twist vectors straight to the modules.
- Apply "Discretization" to accounting for angular movement while translating within the 20ms loop tick. Use `ChassisSpeeds.discretize()` to mathematically offset coordinate skew so the robot actually drives perfectly straight while spinning.

## 3. Loop Safety & Jerk Constraints
- Wrap teleop controls in `SlewRateLimiter` pipelines. The robot accelerates too fast mathematically, which causes real-world wheel slip and odometry loss. Clamping driver acceleration inputs significantly mitigates non-contact odometry drift.
