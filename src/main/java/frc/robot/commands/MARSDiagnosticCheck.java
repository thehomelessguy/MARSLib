package frc.robot.commands;

import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MARSClimber;
import frc.robot.subsystems.MARSCowl;

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
   *   <li>Jolts the Climber at 6.0 Volts for 0.5s to check unloaded lift dynamics.
   *   <li>Returns active test components safely back to zero.
   * </ul>
   *
   * @param swerveDrive The robot drivetrain.
   * @param climber The main lifting/climbing mechanism.
   * @param cowl The shooter hood / adjustable angle mechanism.
   */
  public MARSDiagnosticCheck(SwerveDrive swerveDrive, MARSClimber climber, MARSCowl cowl) {

    addCommands(
        // Ensure standard telemetry begins
        Commands.runOnce(
            () -> DriverStation.reportWarning("Starting Full Diagnostic Sweep...", false)),

        // Swerve Drive Translation Check (0.5m/s for 0.5 sec)
        Commands.run(() -> swerveDrive.runVelocity(new ChassisSpeeds(0.5, 0.0, 0.0)), swerveDrive)
            .withTimeout(0.5),

        // Stop Drivetrain
        Commands.runOnce(
            () -> swerveDrive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), swerveDrive),

        // Check Cowl Pos
        cowl.run(() -> cowl.setTargetPosition(0.2)).withTimeout(0.5),

        // Climbers Check
        Commands.runOnce(() -> climber.setVoltage(6.0), climber),
        Commands.waitSeconds(0.5),
        Commands.runOnce(() -> climber.setVoltage(0.0), climber),

        // Move target positions back to 0
        Commands.runOnce(() -> cowl.setTargetPosition(0.0), cowl),
        Commands.runOnce(() -> climber.setTargetPosition(0.0), climber),
        Commands.runOnce(() -> DriverStation.reportWarning("Diagnostic Check Complete!", false)));
  }
}
