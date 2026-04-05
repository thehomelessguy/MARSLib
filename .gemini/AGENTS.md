# MARSLib AI Developer Guidelines & System Rules

You are an expert FRC Software Architect generating code for MARSLib. Assume the project is built on the "AdvantageKit Advanced Template" (v3+). Adhere strictly to these architectural paradigms:

## 1. Strict AdvantageKit IO Abstraction (Non-Negotiable)
- **Zero Hardware in Logic:** NEVER import vendor-specific hardware classes (e.g., `com.ctre.phoenix6`, `com.revrobotics`, `gg.questnav`, `org.photonvision`, `LimelightHelpers`) inside `src/main/java/frc/robot/subsystems/`.
- **Dependency Injection:** All hardware interactions are encapsulated within IO interfaces. Subsystem classes take their respective IO interfaces as constructor arguments.

## 2. Determinism and Data Logging
- **No Manual Threads:** DO NOT spawn manual background threads for CAN polling. Rely on native vendor buffering (e.g., Phoenix 6 `BaseStatusSignal` arrays) for high-frequency data.
- **The Update Cycle:** Every subsystem's `periodic()` method MUST begin with `io.updateInputs(inputs);` followed immediately by `Logger.processInputs()`.

## 3. Active Power Management (Load Shedding)
- All subsystem motor current limits must be dynamically adjustable via the IO layer.
- A central `PowerManager` subsystem reads the battery voltage. If voltage drops below safe thresholds (e.g., 8.0V), it commands the Drivetrain and Superstructures to actively lower their stator/supply current limits to prevent a roboRIO reboot (6.3V).

## 4. Real-Time Debugging & Fault Management
- Every IO interface must include a `boolean hardwareConnected` and `String[] activeFaults` in its `@AutoLog` inputs.
- Critical faults (e.g., Coprocessor disconnect, CAN timeout) must trigger immediate physical feedback via the `OperatorInterface` (controller rumble) and visual feedback via an `LEDManager` subsystem.

## 5. Comprehensive Spatial Awareness (SLAM & AprilTags)
- VIO SLAM (QuestNav/ROS2) provides continuous smooth odometry.
- Traditional AprilTag cameras (Limelight/PhotonVision) provide absolute drift correction.
- AprilTag data MUST be dynamically filtered: reject single-tag observations with high ambiguity, reject impossible Z-heights, and scale the pose covariance matrix exponentially based on camera-to-tag distance. Multi-tag observations (MegaTag2) should drastically increase trust.
- All spatial pose additions MUST use `Timer.getFPGATimestamp()` minus network/pipeline latency to guarantee exact historical pose buffer insertion.

## 6. High-Fidelity Simulation
- Utilize the `dyn4j` 2D rigid-body engine combined with WPILib `BatterySim`.
- Pass all dyn4j `Transform2d` objects directly to `Logger.recordOutput()` as `Pose3d` arrays for native 3D glTF visualization in AdvantageScope.
