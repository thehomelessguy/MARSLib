package com.marslib.auto;

import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class ShootOnTheMoveCommand extends Command {

  private final SwerveDrive swerveDrive;
  private final DoubleSupplier joystickX;
  private final DoubleSupplier joystickY;
  private final Translation2d targetNode;

  private final PIDController thetaAlignController;

  // Approximate game piece exit velocity (Tune this matching your shooter mechanisms!)
  private static final double PROJECTILE_SPEED_MPS = 15.0;

  /**
   * Overrides the driver's rotational (Theta) control to perfectly track a specific field
   * coordinate using Velocity-Added Kinematic Leading, while allowing them to freely strafe and
   * sprint in X and Y simultaneously.
   *
   * @param swerveDrive Standard drive subsystem hook.
   * @param joystickX Lambda to the driver's X-axis joystick input (Field X).
   * @param joystickY Lambda to the driver's Y-axis joystick input (Field Y).
   * @param targetNode The stationary coordinate point we want to shoot at.
   */
  public ShootOnTheMoveCommand(
      SwerveDrive swerveDrive,
      DoubleSupplier joystickX,
      DoubleSupplier joystickY,
      Translation2d targetNode) {
    this.swerveDrive = swerveDrive;
    this.joystickX = joystickX;
    this.joystickY = joystickY;
    this.targetNode = targetNode;

    this.thetaAlignController = new PIDController(5.0, 0, 0);
    this.thetaAlignController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveDrive);
  }

  @Override
  public void execute() {
    Pose2d currentPose = swerveDrive.getPose();

    // 1. Let the driver keep complete X/Y translating freedom
    double fieldVx = joystickX.getAsDouble();
    double fieldVy = joystickY.getAsDouble();

    // 2. Extract true robot instantaneous momentum for Time-Of-Flight math
    ChassisSpeeds currentSpeeds = swerveDrive.getChassisSpeeds();
    ChassisSpeeds currentFieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());

    // 3. Velocity-Added Kinematic Leading Math (The Magic)
    double distance = currentPose.getTranslation().getDistance(targetNode);
    double timeOfFlight = distance / PROJECTILE_SPEED_MPS;

    // Shift the target backwards opposite to our momentum so the game piece sweeps in perfectly
    double virtualTargetX =
        targetNode.getX() - (currentFieldSpeeds.vxMetersPerSecond * timeOfFlight);
    double virtualTargetY =
        targetNode.getY() - (currentFieldSpeeds.vyMetersPerSecond * timeOfFlight);

    // Calculate heading intercept
    double aimTheta =
        Math.atan2(virtualTargetY - currentPose.getY(), virtualTargetX - currentPose.getX());

    // 4. Automate rotation to track the moving virtual intercept
    double omega = thetaAlignController.calculate(currentPose.getRotation().getRadians(), aimTheta);

    // 5. Package field-centric commands into kinematics
    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldVx, fieldVy, omega, currentPose.getRotation());

    swerveDrive.runVelocity(robotSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.runVelocity(new ChassisSpeeds());
  }
}
