# 🔧 MARSLib Pit Debugging Flowchart — Team 2614

Use this guide when the robot is misbehaving in the pits. Start at the top and follow the arrows.

---

## Master Triage

```mermaid
flowchart TD
    START["🤖 Robot Not Working"] --> Q1{"Does the RoboRIO boot?<br/>(Green LED on RIO)"}

    Q1 -- No --> POWER_TREE["⚡ Go to: POWER TREE"]
    Q1 -- Yes --> Q2{"Does Driver Station<br/>show Comms?"}

    Q2 -- No --> NETWORK_TREE["🌐 Go to: NETWORK TREE"]
    Q2 -- Yes --> Q3{"Does Driver Station<br/>show Code?"}

    Q3 -- No --> CODE_TREE["💻 Go to: CODE TREE"]
    Q3 -- Yes --> Q4{"Can you Enable<br/>the robot?"}

    Q4 -- No --> Q4A{"Check DS for<br/>Watchdog / Brownout<br/>errors"}
    Q4A --> POWER_TREE
    Q4 -- Yes --> Q5{"Does the robot<br/>drive correctly?"}

    Q5 -- No --> MECHANISM_TREE["🦾 Go to: MECHANISM TREE"]
    Q5 -- Yes --> Q6{"Does Autonomous<br/>work?"}

    Q6 -- No --> AUTO_TREE["🏎️ Go to: AUTO TREE"]
    Q6 -- Yes --> DONE["✅ Robot is Nominal!<br/>Run SystemCheckCommand<br/>in Test Mode to verify."]
```

---

## ⚡ Power Tree

```mermaid
flowchart TD
    P1["⚡ POWER TREE"] --> P2{"Is the main<br/>breaker ON?"}
    P2 -- No --> P2A["Flip the breaker ON"]
    P2 -- Yes --> P3{"Battery voltage<br/>> 12.0V?"}

    P3 -- No --> P3A["🔋 Swap to a<br/>freshly charged battery"]
    P3 -- Yes --> P4{"Are all PDP/PDH<br/>breakers seated?"}

    P4 -- No --> P4A["Re-seat the<br/>popped breaker"]
    P4 -- Yes --> P5{"Check for burnt<br/>smell or melted<br/>connectors"}

    P5 -- Yes --> P5A["🚨 STOP. Get a mentor.<br/>Inspect wiring immediately."]
    P5 -- No --> P6{"RIO Status LED<br/>is solid orange?"}

    P6 -- Yes --> P6A["RIO is in Safe Mode.<br/>Re-image the RIO via USB<br/>using the RoboRIO Imaging Tool."]
    P6 -- No --> P7["Verify 12V at VRM<br/>and Radio barrel jack.<br/>Check Anderson connectors."]
```

---

## 🌐 Network Tree

```mermaid
flowchart TD
    N1["🌐 NETWORK TREE"] --> N2{"Is the radio<br/>powered? (LEDs on)"}
    N2 -- No --> N2A["Check 12V barrel jack<br/>from VRM to Radio"]
    N2 -- Yes --> N3{"Laptop connected<br/>to robot WiFi?"}

    N3 -- No --> N3A["Connect via Ethernet<br/>tether for pit debugging"]
    N3 -- Yes --> N4{"Can you ping<br/>10.26.14.2?"}

    N4 -- No --> N4A["RIO IP may have changed.<br/>Re-image or set static IP<br/>via USB connection."]
    N4 -- Yes --> N5{"DS shows Comms<br/>but high latency?"}

    N5 -- Yes --> N5A["Check CAN utilization<br/>in DS Diagnostics tab.<br/>If > 80%, reduce<br/>status frame rates."]
    N5 -- No --> N6["Restart Driver Station<br/>and Robot Code.<br/>If persistent, re-flash radio."]
```

---

## 💻 Code Tree

```mermaid
flowchart TD
    C1["💻 CODE TREE"] --> C2{"Did code deploy<br/>successfully?"}
    C2 -- No --> C2A["Run: ./gradlew deploy<br/>Check console for errors"]
    C2 -- Yes --> C3{"DS shows 'No Robot Code'<br/>after deploy?"}

    C3 -- Yes --> C3A{"Check RIO log via<br/>SSH or Driver Station<br/>console tab"}
    C3A --> C3B{"NullPointerException<br/>or CrashTracker?"}
    C3B -- Yes --> C3C["A subsystem constructor<br/>is crashing. Check<br/>AdvantageScope logs<br/>for the stack trace."]
    C3B -- No --> C3D["Possible dependency issue.<br/>Run: ./gradlew clean build<br/>and redeploy."]

    C3 -- No --> C4{"Code runs but<br/>throws warnings?"}
    C4 -- Yes --> C4A["Check AdvantageScope<br/>Alerts tab for<br/>MARSLib fault messages."]
    C4 -- No --> C5["Code is healthy.<br/>Issue is elsewhere."]
```

---

## 🦾 Mechanism Tree

```mermaid
flowchart TD
    M1["🦾 MECHANISM TREE"] --> M2{"Which mechanism<br/>is failing?"}

    M2 --> M_SWERVE["Swerve Modules"]
    M2 --> M_ELEV["Elevator"]
    M2 --> M_ARM["Arm"]
    M2 --> M_INTAKE["Intake / Shooter"]

    M_SWERVE --> S1{"Modules spinning<br/>but not driving?"}
    S1 -- Yes --> S1A["Absolute encoder offset<br/>is wrong. Recalibrate<br/>CANcoder zero in<br/>SwerveConstants.java"]
    S1 -- No --> S2{"One module dead?"}
    S2 -- Yes --> S2A["Check CAN ID assignment.<br/>Verify TalonFX LED blinks<br/>orange (no signal = CAN wiring)."]
    S2 -- No --> S3{"Robot drives but<br/>drifts sideways?"}
    S3 -- Yes --> S3A["Gyro may need recalibration.<br/>Let robot sit still for<br/>10 seconds on boot.<br/>Check AdvantageScope<br/>SwerveDrive/GyroYaw."]

    M_ELEV --> E1{"Elevator not moving?"}
    E1 -- Yes --> E1A["Run SystemCheckCommand.<br/>If encoder delta = 0,<br/>check belt tension and<br/>motor CAN connection."]
    E1 -- No --> E2{"Elevator oscillating<br/>or overshooting?"}
    E2 -- Yes --> E2A["Reduce kP in<br/>MARSElevator constants.<br/>Verify kG (gravity FF)<br/>is tuned correctly."]

    M_ARM --> A1{"Arm dropping<br/>under gravity?"}
    A1 -- Yes --> A1A["kG feedforward is wrong.<br/>Increase ArmFeedforward.kG<br/>until arm holds position<br/>with PID output near 0."]
    A1 -- No --> A2{"Arm hitting elevator?"}
    A2 -- Yes --> A2A["MARSSuperstructure<br/>collision bounds are wrong.<br/>Check bounding box constants<br/>in Constants.java."]

    M_INTAKE --> I1{"Motor spinning<br/>but no intake?"}
    I1 -- Yes --> I1A["Physical issue:<br/>Check rollers, belts,<br/>and compression distance."]
    I1 -- No --> I2{"Motor not spinning?"}
    I2 -- Yes --> I2A["Check Stator current limit.<br/>If set too low, motor<br/>cannot overcome static<br/>friction. Increase to 40A."]
```

---

## 🏎️ Auto Tree

```mermaid
flowchart TD
    A1["🏎️ AUTO TREE"] --> A2{"Robot doesn't move<br/>in Auto?"}
    A2 -- Yes --> A3{"Is the starting pose<br/>set correctly?"}
    A3 -- No --> A3A["Verify resetPose() is called<br/>at the start of auto<br/>with the correct Pose2d."]
    A3 -- Yes --> A4{"PathPlanner path<br/>loaded?"}
    A4 -- No --> A4A["Check that .path files<br/>exist in deploy/pathplanner.<br/>Verify NamedCommands are<br/>registered in RobotContainer."]
    A4 -- Yes --> A4B["Check AdvantageScope<br/>Auto/ActivePath to see<br/>if the path is being followed."]

    A2 -- No --> A5{"Robot drives but<br/>is off-target?"}
    A5 -- Yes --> A6{"Consistently off<br/>in one direction?"}
    A6 -- Yes --> A6A["Odometry is drifting.<br/>Check wheel radius in<br/>SwerveConstants.java.<br/>Verify vision fusion<br/>standard deviations."]
    A6 -- No --> A6B["PathPlanner PID gains<br/>need tuning. Adjust<br/>Auto/Translation_kP and<br/>Auto/Rotation_kP in<br/>AdvantageScope."]

    A5 -- No --> A7{"Auto works but<br/>game pieces miss?"}
    A7 -- Yes --> A7A["Check ShootOnTheMoveCommand<br/>target coordinates.<br/>Verify alliance origin<br/>flipping in Constants.java."]
```

---

## Quick Reference Checklist

| Step | Action | Time |
|------|--------|------|
| 1 | Swap to a fresh battery (>12.5V) | 30s |
| 2 | Tether via Ethernet, verify Comms | 15s |
| 3 | Deploy code: `./gradlew deploy` | 45s |
| 4 | Enable in Test Mode, run `SystemCheckCommand` | 10s |
| 5 | Check AdvantageScope Alerts tab | 5s |
| 6 | Enable Teleop, verify all axes drive correctly | 15s |
| 7 | Run Auto routine on practice field | 30s |

> [!TIP]
> **Total pit turnaround target: under 3 minutes.** If you can't diagnose in 3 minutes, swap the battery and call a mentor.

> [!CAUTION]
> **NEVER enable the robot on blocks in the pit with mechanisms that could swing or extend.** Always have a spotter and ensure the e-stop is within arm's reach.
