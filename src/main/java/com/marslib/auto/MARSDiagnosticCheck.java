package com.marslib.auto;

import com.marslib.hmi.LEDManager;
import com.marslib.mechanisms.MARSArm;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.mechanisms.MARSShooter;
import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Sequential command group that performs a predefined sequence of automated hardware checks. */
public class MARSDiagnosticCheck extends SequentialCommandGroup {

  /**
   * Constructs a full diagnostic sweep command.
   *
   * <p>Executes a sequenced hardware sweep:
   *
   * <ul>
   *   <li>Runs SwerveDrive forward at 0.5m/s for 0.5s to verify translation blocks.
   *   <li>Moves the Cowl to 0.2 radians.
   *   <li>Jolts the FastClimber at 6.0 Volts for 0.5s to check unloaded lift dynamics.
   *   <li>Returns active test components safely back to zero.
   * </ul>
   *
   * @param swerveDrive The robot drivetrain.
   * @param fastClimber The main lifting/climbing mechanism.
   * @param cowl The shooter hood / adjustable angle mechanism.
   * @param intakePivot The pivoting intake deployment mechanism.
   * @param floorIntake The under-the-bumper floor roller intake.
   * @param shooter The primary flywheel firing subsystem.
   * @param feeder The serializer feeding game pieces into the shooter.
   * @param ledManager The HMI feedback LED controller.
   */
  public MARSDiagnosticCheck(
      SwerveDrive swerveDrive,
      MARSElevator fastClimber,
      MARSArm cowl,
      MARSArm intakePivot,
      MARSShooter floorIntake,
      MARSShooter shooter,
      MARSShooter feeder,
      LEDManager ledManager) {

    addCommands(
        // Ensure standard telemetry begins
        Commands.runOnce(() -> System.out.println("Starting Full Diagnostic Sweep...")),

        // Swerve Drive Translation Check (0.5m/s for 0.5 sec)
        Commands.run(() -> swerveDrive.runVelocity(new ChassisSpeeds(0.5, 0.0, 0.0)), swerveDrive)
            .withTimeout(0.5),

        // Stop Drivetrain
        Commands.runOnce(
            () -> swerveDrive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), swerveDrive),

        // Check Cowl Pos
        cowl.run(() -> cowl.setTargetPosition(0.2)).withTimeout(0.5),

        // Climbers Check
        Commands.runOnce(() -> fastClimber.setVoltage(6.0), fastClimber),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> fastClimber.setVoltage(0.0), fastClimber),

        // Move target positions back to 0
        Commands.runOnce(() -> cowl.setTargetPosition(0.0), cowl),
        Commands.runOnce(() -> fastClimber.setTargetPosition(0.0), fastClimber),
        Commands.runOnce(() -> System.out.println("Diagnostic Check Complete!")));
  }
}
