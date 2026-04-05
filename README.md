# MARSLib Workspace

Welcome to the MARSLib Advanced Simulation and Abstraction Library.

This architecture is designed to enforce consistent AdvantageKit input abstraction across hardware components. It enables immediate, real-time physics simulations of robot mechanisms (Elevators, Arms, Swerve Drivetrains) using Dyn4J while still allowing high-performance mapping to real hardware devices (e.g. TalonFX/CANSparkMax).

## Project Structure

Your logic is segmented into two spheres: 

1. `com.marslib.*`: This is the inner library. It contains the abstraction interfaces (`LinearMechanismIO`, `RotaryMechanismIO`), the Dyn4j physics simulation wrappers (`LinearMechanismIOSim`, `RotaryMechanismIOSim`), Fault Management (`MARSFaultManager`), HMI interfaces, and Vision interfaces.
   - Do **NOT** write specific robot logic here. This is the hardware engine.
2. `frc.robot.*`: This is the standard WPILib competition code footprint. You will create your Subsystems, Commands, and RobotContainer directly in here. 

## How to use Subsystems

MARSLib uses AdvantageKit IO layers. When writing a new subsystem, you will create:
1. `MySubsystemIO` (Interface defining the inputs like `position`, `velocity`, `currentAmps`)
2. `MySubsystemIOReal` (Reads actual CAN bus data from physical motors)
3. `MySubsystemIOSim` (Optionally use MARSLib's Dyn4j wrappers if available, or write custom physics)
4. `MySubsystem` (The SubsystemBase class that takes `MySubsystemIO` in its constructor and executes control logic)

## Simulating the Robot

Because MARSLib overrides the IO layer, running `Simulate Robot` in VS Code/Antigravity will automatically bridge the subsystem commands into `MARSPhysicsWorld`. AdvantageScope can then connect via NetworkTables to visualize the robot's odometry and states instantly.

## Building and Deploying

- **Build Code:** `./gradlew build`
- **Deploy to Robot:** `./gradlew deploy`
- **Run Simulation:** `./gradlew simulateJava`
