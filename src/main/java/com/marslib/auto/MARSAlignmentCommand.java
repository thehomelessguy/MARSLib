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

  private final double translationTolerance;
  private final double rotationTolerance;

  /** Constructs an alignment command with parameterized constants. */
  public MARSAlignmentCommand(
      SwerveDrive swerveDrive,
      Supplier<Pose2d> targetPoseSupplier,
      double kPTranslation,
      double kPRotation,
      double maxTranslationVelMps,
      double maxTranslationAccelMps2,
      double maxRotationVelRps,
      double maxRotationAccelRps2,
      double translationToleranceMeters,
      double rotationToleranceRad) {

    this.swerveDrive = swerveDrive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.translationTolerance = translationToleranceMeters;
    this.rotationTolerance = rotationToleranceRad;

    TrapezoidProfile.Constraints translationConstraints =
        new TrapezoidProfile.Constraints(maxTranslationVelMps, maxTranslationAccelMps2);
    TrapezoidProfile.Constraints rotationConstraints =
        new TrapezoidProfile.Constraints(maxRotationVelRps, maxRotationAccelRps2);

    xController = new ProfiledPIDController(kPTranslation, 0, 0, translationConstraints);
    yController = new ProfiledPIDController(kPTranslation, 0, 0, translationConstraints);

    thetaController = new ProfiledPIDController(kPRotation, 0, 0, rotationConstraints);
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

    return error.getNorm() < translationTolerance && rotationError < rotationTolerance;
  }

  @Override
  public void end(boolean interrupted) {
    swerveDrive.runVelocity(new ChassisSpeeds(0, 0, 0));
  }
}
