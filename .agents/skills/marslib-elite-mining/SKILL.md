---
name: marslib-elite-mining
description: Helps ingest, analyze, and port advanced code from World Champion and Elite FRC teams directly into the MARSLib architecture. Use when extracting custom trajectory math, state-machine layouts, vision fusion systems, user interfaces, data logging, control frameworks, and any other sub-systems you'd program for an FRC robot. Validates and translates from specific high-performing repositories like 1690, 254, 6328, 2910, etc.
---

You are an expert repository mining engineer for Team MARS. When extracting custom trajectory math, state-machine layouts, vision fusion systems, or control frameworks from Elite FRC teams, adhere strictly to the following guidelines.

# MARSLib Elite Repository Mining Agent

This skill dictates how to safely clone, parse, translate, and securely integrate logic from Elite FRC team implementations into the `.MARSLib` infrastructure without degrading our code style or creating arbitrary hardware couplings.
**CRITICAL RULE:** Do NOT under any circumstances use `search_web` tools to find repositories. You must rely EXCLUSIVELY on the static manifest of GitHub URLs provided below. Clone or query these URLs directly.

## 1. Top Tier Manifest (The Elite Hit List)

### FRC Season Reference Table
Use this table to map common game names to their respective competition years when parsing repository names.

| Year | FRC Game Name |
| :--- | :--- |
| **2026** | Rebuilt |
| **2025** | Reefscape |
| **2024** | Crescendo |
| **2023** | Charged Up |
| **2022** | Rapid React |
| **2021** | Infinite Recharge (At Home) |
| **2020** | Infinite Recharge |
| **2019** | Destination: Deep Space |
| **2018** | FIRST Power Up |

When tasked with "seeing how X team solved Y problem," use the following catalog to inform your specific GitHub URLs:

*   **Team 6328 (Mechanical Advantage):** Core *AdvantageKit* creators. Use for evaluating logging abstraction structures.
    *   2026: `https://github.com/Mechanical-Advantage/RobotCode2026Public`
    *   2025: `https://github.com/Mechanical-Advantage/RobotCode2025Public`
    *   2024: `https://github.com/Mechanical-Advantage/RobotCode2024Public`
    *   2023: `https://github.com/Mechanical-Advantage/RobotCode2023`
    *   2022: `https://github.com/Mechanical-Advantage/RobotCode2022`
    *   2020: `https://github.com/Mechanical-Advantage/RobotCode2020`
*   **Team 254 (The Cheesy Poofs):** Pioneers of arbitrary *Path Following* and *State-Space*. Use for custom math controllers and pure-pursuit trajectory logic.
    *   2025: `https://github.com/Team254/FRC-2025-Public`
    *   2024: `https://github.com/Team254/FRC-2024-Public`
    *   2023: `https://github.com/Team254/FRC-2023-Public`
    *   2022: `https://github.com/Team254/FRC-2022-Public`
    *   2020: `https://github.com/Team254/FRC-2020-Public`
*   **Team 1690 (Orbit):** Elite *Targeting* and *Continuous Motion*. Use for shoot-on-the-move math and high-speed multi-subsystem orchestration.
    *   2024: `https://github.com/Orbit-Robotics/2024-Robot`
*   **Team 2910 (Jack in the Bot):** *Swerve Architecture* pioneers. Use to analyze SDS motor/encoder geometric models.
    *   2025: `https://github.com/FRCTeam2910/2025CompetitionRobot-Public`
    *   2024: `https://github.com/FRCTeam2910/2024CompetitionRobot-Public`
    *   2023: `https://github.com/FRCTeam2910/2023CompetitionRobot-Public`
    *   2022: `https://github.com/FRCTeam2910/2022CompetitionRobot`
    *   2021: `https://github.com/FRCTeam2910/2021CompetitionRobot`
    *   2020: `https://github.com/FRCTeam2910/2020CompetitionRobot`
*   **Team 1678 (Citrus Circuits):** *State-Machine Design* & System Reliability. Use for robust sequencing maps.
    *   2025: `https://github.com/frc1678/C2025-Public`
    *   2024: `https://github.com/frc1678/C2024-Public`
    *   2023: `https://github.com/frc1678/C2023-Public`
    *   2022: `https://github.com/frc1678/C2022-Public`
    *   2021: `https://github.com/frc1678/cardinal-2021-public`
    *   2020: `https://github.com/frc1678/C2020`
*   **Team 3005 (RoboChargers):** *AdvantageKit Native* & highly clean abstractions.
    *   2025: `https://github.com/FRC3005/Reefscape-2025`
    *   2024: `https://github.com/FRC3005/Crescendo-2024-Public`
    *   2023: `https://github.com/FRC3005/Charged-Up-2023-Public`
    *   2022: `https://github.com/FRC3005/Rapid-React-2022-Public`
    *   2020: `https://github.com/FRC3005/Infinite-Recharge-2020`
*   **Team 364 (BaseFalconSwerve):**
    *   Template: `https://github.com/Team364/BaseFalconSwerve`
*   **Team 111 (WildStang):** Exceptional Component-Based Architecture & clean abstractions.
    *   2026: `https://github.com/wildstang/2026_111_robot_software`
    *   2025: `https://github.com/wildstang/2025_111_robot_software`
    *   2024: `https://github.com/wildstang/2024_111_robot_software`
    *   2023: `https://github.com/wildstang/2023_111_robot_software`
    *   2022: `https://github.com/wildstang/2022_111_robot_software`
    *   2020: `https://github.com/wildstang/2020_robot_software`
*   **Team 2767 (Stryke Force):** 'ThirdCoast' Custom Swerve Framework & advanced control math.
    *   ThirdCoast Framework: `https://github.com/strykeforce/thirdcoast`
    *   2024 (Crescendo): `https://github.com/strykeforce/crescendo`
    *   2023 (Charged Up): `https://github.com/strykeforce/chargedup`
    *   2022 (Rapid React): `https://github.com/strykeforce/rapidreact`
    *   2020 (Infinite Recharge): `https://github.com/strykeforce/infiniterecharge`
*   **Team 5940 (B.R.E.A.D.):** Bleeding-Edge Vision Fusion, AdvantageKit usage, & Odometry Hardening.
    *   2025: `https://github.com/BREAD5940/2025-Public`
    *   2024: `https://github.com/BREAD5940/2024-Onseason`
    *   2023: `https://github.com/BREAD5940/2023-Onseason`
    *   2022: `https://github.com/BREAD5940/2022-Onseason`
    *   2021: `https://github.com/BREAD5940/2021-Robot`
    *   2020: `https://github.com/BREAD5940/2020-Onseason`
*   **Team 604 (Quixilver):** Highly Object-Oriented Simulation & robust WPILib Architecture.
    *   2025: `https://github.com/frc604/2025-public`
    *   2024: `https://github.com/frc604/2024-public`
    *   2023: `https://github.com/frc604/2023-public`
    *   2022: `https://github.com/frc604/2022-public`
    *   2020: `https://github.com/frc604/FRC-2021-v2`
*   **Team 973 (Greybots):** Competition-Tested Superstructure State Machines & robust mechanical integration.
    *   2025: `https://github.com/Team973/2025-inseason`
    *   2023: `https://github.com/Team973/2023-inseason`
    *   2022: `https://github.com/Team973/2022-inseason`
*   **Other Notables (AdvantageKit & Cycles):**
    *   Team 125: `https://github.com/nutrons`
    *   Team 4099: `https://github.com/Team4099`
    *   Team 1323: `https://github.com/Team1323`

## 2. Ingestion Rules (Safety First)

Do **NOT** clone external elite code directly into the workspace root.
*   **Multi-Team Sourcing:** You must ALWAYS attempt to ingest and analyze code from at least TWO DIFFERENT TEAMS (whenever applicable) for any given architectural or implementation question, rather than relying on a single source of truth.
*   **Exhaustive Search & Follow-Up:** If you cannot find a satisfactory answer or implementation within the initially cloned repositories, you MUST execute a follow-up action: autonomously expand your search to additional teams on the static manifest. Do not stop at the first failure.
*   **Targeting a Specific Year:** FRC teams typically create a new repository for each season (e.g., `Robot-2024`, `ChargedUp`, `Crescendo`). If the user asks to investigate a given year's code, you must guess or use github tools on the organization URL to find the exact repository URL for that specific year before cloning.
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
