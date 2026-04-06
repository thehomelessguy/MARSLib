package com.marslib.auto;

import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

/**
 * Overrides the driver's rotational (Theta) control to perfectly track a specific field coordinate
 * using Velocity-Added Kinematic Leading, while allowing them to freely strafe and sprint in X and
 * Y simultaneously.
 */
public class ShootOnTheMoveCommand extends Command {

  private final SwerveDrive swerveDrive;
  private final DoubleSupplier joystickX;
  private final DoubleSupplier joystickY;

  private final PIDController thetaAlignController;

  // Approximate game piece exit velocity (Tune this matching your shooter mechanisms!)
  public ShootOnTheMoveCommand(
      SwerveDrive swerveDrive, DoubleSupplier joystickX, DoubleSupplier joystickY) {
    this.swerveDrive = swerveDrive;
    this.joystickX = joystickX;
    this.joystickY = joystickY;

    this.thetaAlignController =
        new PIDController(frc.robot.Constants.AutoConstants.ALIGN_THETA_KP, 0, 0);
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

    // Determine dynamic target based on alliance
    Translation2d targetNode = frc.robot.Constants.FieldConstants.BLUE_HUB_POS;
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      targetNode = frc.robot.Constants.FieldConstants.RED_HUB_POS;
    }

    // 3. Exact True-Vector Quadratic Time-Of-Flight Intersection Solver
    double dx = targetNode.getX() - currentPose.getX();
    double dy = targetNode.getY() - currentPose.getY();
    double vx = currentFieldSpeeds.vxMetersPerSecond;
    double vy = currentFieldSpeeds.vyMetersPerSecond;
    double s = frc.robot.Constants.ShooterConstants.PROJECTILE_SPEED_MPS;

    double a = (s * s) - ((vx * vx) + (vy * vy));
    double b = -2.0 * ((dx * vx) + (dy * vy));
    double c = -((dx * dx) + (dy * dy));

    double discriminant = (b * b) - (4.0 * a * c);
    double timeOfFlight;

    if (discriminant < 0.0 || a == 0.0) {
      // Fallback to naive approximation if physically impossible to mathematically solve
      timeOfFlight = currentPose.getTranslation().getDistance(targetNode) / Math.max(s, 0.01);
    } else {
      double t1 = (-b + Math.sqrt(discriminant)) / (2.0 * a);
      double t2 = (-b - Math.sqrt(discriminant)) / (2.0 * a);

      if (t1 > 0.0 && t2 > 0.0) timeOfFlight = Math.min(t1, t2);
      else if (t1 > 0.0) timeOfFlight = t1;
      else if (t2 > 0.0) timeOfFlight = t2;
      else timeOfFlight = currentPose.getTranslation().getDistance(targetNode) / Math.max(s, 0.01);
    }

    // Shift the target backwards opposite to our momentum so the game piece sweeps in perfectly
    double virtualTargetX = targetNode.getX() - (vx * timeOfFlight);
    double virtualTargetY = targetNode.getY() - (vy * timeOfFlight);

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
