---
name: marslib-power
description: Helps regulate electrical hardware configurations, CAN bus utilization, and Stator/Supply constraints.
---

# MARSLib Power & Hardware Skill

You are an electrical engineering master in FRC. When configuring motor thresholds and networking:

## 1. Stator vs. Supply Current
Always configure both on TalonFX/SparkMax devices:
- **Supply Current:** Limits the raw amperage drawn from the battery (e.g., `40A`). Set this to physically prevent 120A total system pull and stop RoboRIO brownouts.
- **Stator Current:** Limits the magnetic torque output (e.g., `60A`). Set this to cap absolute mechanical push-force, preventing mechanism breakage or carpet tearing.

## 2. CAN Bus Utilization
- Limit non-critical signals instantly upon configuration. If a status frame (like Phase Voltage or Motor Temp) is not actively used in a closed-loop control algorithm, drop its frequency to `4Hz` or `10Hz` using `BaseStatusSignal.setUpdateFrequency()`.
- Over 80% CAN utilization is a failure. Always optimize.

## 3. Load Shedding & Brownout Protection
- Monitor the global system Voltage. If `RobotController.getBatteryVoltage() < 7.5V`, actively throttle non-vital mechanism targets (e.g., lower Intake target RPM, drop LED brightness) to leave overhead for the Swerve Drive modules to keep iterating computationally without rebooting the RIO.
