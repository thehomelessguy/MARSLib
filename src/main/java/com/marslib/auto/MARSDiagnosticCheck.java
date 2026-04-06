package com.marslib.auto;

import com.marslib.hmi.LEDManager;
import com.marslib.mechanisms.MARSArm;
import com.marslib.mechanisms.MARSElevator;
import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MARSDiagnosticCheck extends SequentialCommandGroup {

  public MARSDiagnosticCheck(
      SwerveDrive swerveDrive, MARSElevator elevator, MARSArm arm, LEDManager ledManager) {

    addCommands(
        Commands.runOnce(() -> System.out.println("Starting MARS System Diagnostic Check")),

        // Swerve Drive Translation Check (0.5m/s for 0.5 sec)
        Commands.run(() -> swerveDrive.runVelocity(new ChassisSpeeds(0.5, 0.0, 0.0)), swerveDrive)
            .withTimeout(0.5),
        Commands.run(() -> swerveDrive.runVelocity(new ChassisSpeeds(-0.5, 0.0, 0.0)), swerveDrive)
            .withTimeout(0.5),

        // Swerve Rotation Check (1.0 rad/s for 0.5 sec)
        Commands.run(() -> swerveDrive.runVelocity(new ChassisSpeeds(0.0, 0.0, 1.0)), swerveDrive)
            .withTimeout(0.5),

        // Stop Drivetrain
        Commands.runOnce(
            () -> swerveDrive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), swerveDrive),

        // Move Elevator Check
        elevator.run(() -> elevator.setTargetPosition(0.2)).withTimeout(0.5),

        // Move Arm Check
        arm.run(() -> arm.setTargetPosition(0.2)).withTimeout(0.5),

        // Return Superstructure to Stow
        Commands.runOnce(() -> arm.setTargetPosition(0.0), arm),
        Commands.waitUntil(() -> Math.abs(arm.getPositionRads()) < 0.05).withTimeout(2.0),
        Commands.runOnce(() -> elevator.setTargetPosition(0.0), elevator),
        Commands.runOnce(() -> System.out.println("Diagnostic Check Complete!")));
  }
}
