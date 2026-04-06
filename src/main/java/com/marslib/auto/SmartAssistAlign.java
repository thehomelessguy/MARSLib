package com.marslib.auto;

import com.marslib.swerve.SwerveConstants;
import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class SmartAssistAlign extends Command {

  private final SwerveDrive swerveDrive;
  private final DoubleSupplier forwardThrottleSupplier;
  private final Pose2d targetNode;

  private final PIDController yAlignController;
  private final PIDController thetaAlignController;

  /**
   * Overrides the driver's lateral (Y) and rotational (Theta) control to perfectly track a specific
   * field coordinate, while allowing them to maintain forward/backward (X) speed.
   *
   * @param swerveDrive Standard drive subsystem hook.
   * @param forwardThrottleSupplier Lambda to the driver's X-axis joystick input.
   * @param targetNode The stationary coordinate mapping we want to align to.
   */
  public SmartAssistAlign(
      SwerveDrive swerveDrive, DoubleSupplier forwardThrottleSupplier, Pose2d targetNode) {
    this.swerveDrive = swerveDrive;
    this.forwardThrottleSupplier = forwardThrottleSupplier;
    this.targetNode = targetNode;

    // These controllers compare the Robot's true position to the Node's true position
    this.yAlignController = new PIDController(4.0, 0, 0);
    this.thetaAlignController = new PIDController(5.0, 0, 0);
    this.thetaAlignController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveDrive);
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerveDrive.getPose();

    // 1. Let the driver keep control of the speed advancing towards the grid (Field X)
    double fieldVx = forwardThrottleSupplier.getAsDouble() * SwerveConstants.MAX_LINEAR_SPEED_MPS;

    // 2. Automate strafing to line up exactly with target Y (Field Y)
    double fieldVy = yAlignController.calculate(currentPose.getY(), targetNode.getY());

    // 3. Automate rotation to point exactly at target heading (Field Theta)
    double omega =
        thetaAlignController.calculate(
            currentPose.getRotation().getRadians(), targetNode.getRotation().getRadians());

    // 4. Critically translate these Field-Relative targets into Robot-Centric kinematics
    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldVx, fieldVy, omega, currentPose.getRotation());

    // Feed to kinematics
    swerveDrive.runVelocity(robotSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.runVelocity(new ChassisSpeeds());
  }
}
