---
name: marslib-elite-mining
description: Helps ingest, analyze, and port advanced code from World Champion and Elite FRC teams directly into the MARSLib architecture. Use when extracting custom trajectory math, state-machine layouts, vision fusion systems, user interfaces, data logging, control frameworks, and any other sub-systems you'd program for an FRC robot. Validates and translates from specific high-performing repositories like 1690, 254, 6328, 2910, etc.
---

# MARSLib Elite Repository Mining Agent

This skill dictates how to safely clone, parse, translate, and securely integrate logic from Elite FRC team implementations into the `.MARSLib` infrastructure without degrading our code style or creating arbitrary hardware couplings.

## 1. Top Tier Manifest (The Elite Hit List)

When tasked with "seeing how X team solved Y problem," use the following catalog to inform your specific GitHub URLs:

*   **Team 6328 (Mechanical Advantage):** `https://github.com/Mechanical-Advantage` - Core *AdvantageKit* creators. Use for evaluating logging abstraction structures.
*   **Team 254 (The Cheesy Poofs):** `https://github.com/Team254` - Pioneers of arbitrary *Path Following* and *State-Space*. Use for custom math controllers and pure-pursuit trajectory logic.
*   **Team 1690 (Orbit):** `https://github.com/Orbit-Robotics` - Elite *Targeting* and *Continuous Motion*. Use for shoot-on-the-move math and high-speed multi-subsystem orchestration.
*   **Team 2910 (Jack in the Bot):** `https://github.com/FRCTeam2910` - *Swerve Architecture*. Use to analyze SDS motor/encoder geometric models.
*   **Team 1678 (Citrus Circuits):** `https://github.com/frc1678` - *State-Machine Design*. Use for robust sequencing maps.
*   **Team 3005 (RoboChargers):** `https://github.com/frc3005` - *AdvantageKit Native*.
*   **Team 125 (NUTRONs):** `https://github.com/nutrons` - *AdvantageKit Native*.
*   **Team 4099 (The Falcons):** `https://github.com/Team4099` - *AdvantageKit Native*.
*   **Team 364 (BaseFalconSwerve):** `https://github.com/Team364` - *Phoenix Swerve Architectures*.
*   **Team 1323 (MadTown Robotics):** `https://github.com/Team1323` - *Cycle Optimization*.
*   **Team 3476 (Code Orange):** `https://github.com/Team3476` - *Advanced WPILib Extensions*.

## 2. Ingestion Rules (Safety First)

Do **NOT** clone external elite code directly into the workspace root.
*   **Multi-Team Sourcing:** You must ALWAYS attempt to ingest and analyze code from at least TWO DIFFERENT TEAMS (whenever applicable) for any given architectural or implementation question, rather than relying on a single source of truth.
*   **Exhaustive Search & Follow-Up:** If you cannot find a satisfactory answer or implementation within the initially cloned repositories, you MUST execute a follow-up action: autonomously expand your search to additional teams on the manifest, or prompt the user with a plan to use `search_web` to discover other relevant elite repositories. Do not stop at the first failure.
*   **Targeting a Specific Year:** FRC teams typically create a new repository for each season (e.g., `Robot-2024`, `ChargedUp`, `Crescendo`). If the user asks to investigate a given year's code, you must first search the team's GitHub organization (or use `search_web`) to find the exact repository URL for that specific year before cloning.
*   **Isolated Cloning:** Always execute an automated `git clone --depth 1 [EXACT_YEAR_REPO_URL] <appDataDir>\brain\<conversation-id>/scratch/[TEAM_NAME]_[YEAR]` to create an isolated sandbox to read from.

## 3. Pattern Matching Heuristics (Astute Grepping)

Top-tier teams have notoriously large repositories. Avoid getting lost by anchoring your grep searches around key API landmarks:
*   Search for `SwerveModuleState`, `ChassisSpeeds`, or `Phoenix6` logic when auditing drivetrain movement.
*   Search for `PoseEstimator`, `Vision`, `LimelightHelpers`, or `PhotonCamera` when identifying localization math.
*   Search for `StateSpace`, `Matrix`, `LQR`, or `EKF` when looking for pure control theory loop structures.
*   Search for `Dashboard`, `Shuffleboard`, `NetworkTable`, `AdvantageScope`, or `Elastic` when investigating User Interfaces and telemetry layouts.
*   Search for `Logger`, `AdvantageKit`, `AutoLog`, or `DataLogManager` when parsing data logging and telemetry backends.
*   Search for `Controller`, `HID`, `Joystick`, `Haptic`, or `Rumble` when investigating operator input and control systems.
*   **Cross-Team Issue Analysis:** If asked to see how all teams solved a *specific issue* for a given year (e.g. "how did teams score in the Trap in 2024?"), you must clone the relevant yearly repositories for *multiple* teams from the manifest. Systematically grep across all of them for game-specific keywords (e.g., `Trap`, `Elevator`, `Score`) and synthesize a comparative analysis of their differing approaches.
*   *Mandate Cross-Referencing:* If you identify a game-changing Vision configuration in 1690's repository, check it against 6328's approach before considering it generalized best-practice.
*   *WPILib Validation:* If you discover a heavily modified WPILib wrapper (like `CodeOrangePoseEstimator`), you must trace the source back upstream (`github.com/wpilibsuite/allwpilib`) to verify exactly what WPILib limitations caused the team to fork the math.

## 4. Porting Constraints (Absolute Architecture Enforcements)

Extracting elite code is useless if it creates technical debt. Any porting attempt into `MARSLib` must rigidly comply with the following translations:

1.  **Dependency Injection Only:** Discard all singletons (e.g. `Drive.getInstance()`) and hardware instantiation logic. Translate elite calculations to operate entirely inside `MARSLib`'s `@AutoLog` generic `HardwareIO` interfaces.
2.  **No Vendor Lock-in API bleed:** Eliminate direct TalonFX/SparkMAX calls from your final ported snippet. Replace them simply with math variables (e.g., Target Volts or System States), passing them downward into MARSLib `runCharacterization()` or `runVelocity()` layers.
3.  **Strict Variable Formatting:** Nuke legacy `m_` prefixes. Everything must cleanly translate into unannotated `camelCase` parameters standardizing to modern Java layout.
4.  **Logging Normalization:** Convert `SmartDashboard`, `ShuffleBoard` or raw custom JSON REST usages natively into AdvantageKit `LogTable` or auto-logging outputs.

## 5. Exit Validation

Before surfacing the ported architectural snippet up to the user:
*   Validate `spotlessApply` compliance across the local `MARSLib` Gradle context.
*   Confirm there are no unresolved static imports or `HidingField` errors caused by variable conversion mismatching.
