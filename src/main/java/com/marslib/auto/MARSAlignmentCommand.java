package com.marslib.auto;

import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/**
 * Command to bypass default PathPlanner routing and dynamically align the robot natively onto a
 * dynamically moving or stationary field target timestamped via Vision.
 */
public class MARSAlignmentCommand extends Command {
  private final SwerveDrive swerveDrive;
  private final Supplier<Pose2d> targetPoseSupplier;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  public MARSAlignmentCommand(SwerveDrive swerveDrive, Supplier<Pose2d> targetPoseSupplier) {
    this.swerveDrive = swerveDrive;
    this.targetPoseSupplier = targetPoseSupplier;

    // Default fast-response kinematic constraints for visual alignment
    TrapezoidProfile.Constraints translationConstraints =
        new TrapezoidProfile.Constraints(3.0, 3.0);
    TrapezoidProfile.Constraints rotationConstraints =
        new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 4);

    xController = new ProfiledPIDController(5.0, 0, 0, translationConstraints);
    yController = new ProfiledPIDController(5.0, 0, 0, translationConstraints);

    thetaController = new ProfiledPIDController(5.0, 0, 0, rotationConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = swerveDrive.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d target = targetPoseSupplier.get();
    Pose2d currentPos = swerveDrive.getPose();

    double xFeedback = xController.calculate(currentPos.getX(), target.getX());
    double yFeedback = yController.calculate(currentPos.getY(), target.getY());
    double thetaFeedback =
        thetaController.calculate(
            currentPos.getRotation().getRadians(), target.getRotation().getRadians());

    ChassisSpeeds dynamicSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFeedback, yFeedback, thetaFeedback, currentPos.getRotation());

    swerveDrive.runVelocity(dynamicSpeeds);
  }

  @Override
  public boolean isFinished() {
    Pose2d target = targetPoseSupplier.get();
    Pose2d currentPos = swerveDrive.getPose();

    Translation2d error = target.getTranslation().minus(currentPos.getTranslation());
    double rotationError =
        Math.abs(currentPos.getRotation().minus(target.getRotation()).getRadians());

    return error.getNorm() < 0.05 && rotationError < 0.05;
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
