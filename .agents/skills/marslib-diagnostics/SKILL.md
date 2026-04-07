---
name: marslib-diagnostics
description: Helps architect pre-match system checks, hardware sweep assertions, and CAN bus dropout tracking. Use when adding health checks, creating fault alerts, or implementing pre-match test routines.
---

# MARSLib Diagnostics & Health Skill

You are a systems reliability engineer for Team MARS 2614. When writing fault detection or pre-match checks:

## 1. Architecture

The diagnostics system spans two packages:

| Class | Package | Purpose |
|---|---|---|
| `MARSFaultManager` | `com.marslib.faults` | Static registry of critical/warning faults. Tracks new faults since last check. |
| `Alert` | `com.marslib.faults` | AdvantageScope-compatible alert with severity levels (INFO, WARNING, CRITICAL) |
| `MARSDiagnosticCheck` | `com.marslib.auto` | Pre-match sequential system sweep command |
| `SystemCheckCommand` | `frc.robot.commands` | Robot-specific test mode routines |

### Fault Flow
```
Hardware dropout → IO layer detects StatusCode error → MARSFaultManager.registerCriticalFault()
                                                     → Alert("Motor Disconnected", CRITICAL).set(true)
                                                     → LEDManager shows fault flash
```

## 2. Key Rules

### Rule A: Never Crash on Hardware Failure
If a motor drops off CAN mid-match, the IO layer MUST catch the Phoenix 6 `StatusCode` error, log it via `MARSFaultManager`, and continue operating. A dead intake must NOT crash the swerve drivetrain.

### Rule B: Alerts Use MARSLib's Custom Alert Class
Use `com.marslib.faults.Alert`, NOT `edu.wpi.first.wpilibj.Alert`. MARSLib's version has `resetAll()` for test cleanup. If you use WPILib's Alert, test suites will leak state.

### Rule C: Faults Are Static — Reset in Tests
`MARSFaultManager` and `Alert` both use static state. EVERY test `@BeforeEach` block MUST call:
```java
Alert.resetAll();
MARSFaultManager.clear();
```
Failing to do this causes fault state to bleed across tests, producing false failures.

### Rule D: System Sweeps Test Physical Motion
`MARSDiagnosticCheck` doesn't just check connectivity — it commands mechanisms to physical positions and asserts encoder deltas match expected travel. If the elevator is commanded to 0.5m but only reads 0.01m, the gearbox is stripped.

## 3. Adding New Fault Checks

1. In the IO layer's `updateInputs()`, check for hardware errors:
   ```java
   if (driveStatus.getStatus() != StatusCode.OK) {
       MARSFaultManager.registerCriticalFault("Module" + id + " drive motor CAN dropout");
       new Alert("Module " + id + " Drive CAN", AlertType.kError).set(true);
   }
   ```
2. For boot-time checks, add firmware version validation in the IO constructor.
3. For pre-match sweeps, add a new stage to `SystemCheckCommand`:
   ```java
   // Stage: Climber travel test
   Commands.sequence(
       climber.setTargetCommand(0.5),
       Commands.waitSeconds(2.0),
       Commands.runOnce(() -> assertTrue(climber.getPosition() > 0.4))
   );
   ```

## 4. Command API
```java
// In RobotContainer, bind to Test mode:
if (RobotBase.isSimulation()) {
    new Trigger(DriverStation::isTest)
        .onTrue(new SystemCheckCommand(drive, elevator, arm));
}
```

## 5. Telemetry
- `Diagnostics/HasCriticalFaults` — Boolean: any critical fault active
- `Diagnostics/NewCriticalFault` — Most recent fault string
- `Diagnostics/TotalFaultCount` — Cumulative fault count
- `Alerts/*` — Individual alert states visible in AdvantageScope Alerts tab
- `SystemCheck/Stage` — Current diagnostic stage name
- `SystemCheck/Passed` — Boolean: all stages passed
