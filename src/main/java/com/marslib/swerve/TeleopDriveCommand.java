package com.marslib.swerve;

import com.marslib.auto.GhostManager;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Command to handle Teleop Swerve Driving.
 *
 * <p>Evaluates joysticks using the TeleopDriveMath.computeFieldRelativeSpeeds logic for
 * field-oriented control and executes trajectory adjustments based on current inputs and deadbands.
 * Also handles conditional heading lock implementation.
 */
public class TeleopDriveCommand extends Command {
  private final SwerveDrive swerveDrive;
  private final GhostManager ghostManager;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;

  private final SlewRateLimiter xLimiter =
      new SlewRateLimiter(Constants.DriveConstants.TELEOP_LINEAR_ACCEL_LIMIT);
  private final SlewRateLimiter yLimiter =
      new SlewRateLimiter(Constants.DriveConstants.TELEOP_LINEAR_ACCEL_LIMIT);
  private final SlewRateLimiter omegaLimiter =
      new SlewRateLimiter(Constants.DriveConstants.TELEOP_OMEGA_ACCEL_LIMIT);
  private final PIDController headingController =
      new PIDController(Constants.DriveConstants.HEADING_KP, 0, 0);

  private Rotation2d targetHeading = new Rotation2d();
  private final ChassisSpeeds targetSpeeds = new ChassisSpeeds();

  public TeleopDriveCommand(
      SwerveDrive swerveDrive,
      GhostManager ghostManager,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    this.swerveDrive = swerveDrive;
    this.ghostManager = ghostManager;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;

    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    targetHeading = swerveDrive.getPose().getRotation();
  }

  @Override
  public void execute() {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    double rawX = xSupplier.getAsDouble();
    double rawY = ySupplier.getAsDouble();
    double rawOmega = omegaSupplier.getAsDouble();

    ChassisSpeeds preSlewSpeeds =
        TeleopDriveMath.computeFieldRelativeSpeeds(
            ghostManager.getLeftY(() -> rawX),
            ghostManager.getLeftX(() -> rawY),
            ghostManager.getRightX(() -> rawOmega),
            isRed);

    double xVal =
        MathUtil.applyDeadband(ghostManager.getLeftY(() -> rawX), TeleopDriveMath.DEADBAND);
    double yVal =
        MathUtil.applyDeadband(ghostManager.getLeftX(() -> rawY), TeleopDriveMath.DEADBAND);
    double omgVal =
        MathUtil.applyDeadband(ghostManager.getRightX(() -> rawOmega), TeleopDriveMath.DEADBAND);

    double finalX = xLimiter.calculate(preSlewSpeeds.vxMetersPerSecond);
    double finalY = yLimiter.calculate(preSlewSpeeds.vyMetersPerSecond);

    targetSpeeds.vxMetersPerSecond = finalX;
    targetSpeeds.vyMetersPerSecond = finalY;

    if (Math.abs(omgVal) <= 0.01) {
      if (Math.abs(xVal) > 0.01 || Math.abs(yVal) > 0.01) {
        targetSpeeds.omegaRadiansPerSecond =
            headingController.calculate(
                swerveDrive.getPose().getRotation().getRadians(), targetHeading.getRadians());
      } else {
        targetHeading = swerveDrive.getPose().getRotation();
        targetSpeeds.omegaRadiansPerSecond = 0.0;
      }
    } else {
      targetHeading = swerveDrive.getPose().getRotation();
      targetSpeeds.omegaRadiansPerSecond =
          omegaLimiter.calculate(preSlewSpeeds.omegaRadiansPerSecond);
    }

    ChassisSpeeds robotRelativeSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            targetSpeeds.vxMetersPerSecond,
            targetSpeeds.vyMetersPerSecond,
            targetSpeeds.omegaRadiansPerSecond,
            swerveDrive.getPose().getRotation());

    Logger.recordOutput("Teleop/RawJoystickX", rawX);
    Logger.recordOutput("Teleop/RawJoystickY", rawY);
    Logger.recordOutput("Teleop/RawJoystickOmega", rawOmega);
    Logger.recordOutput("Teleop/PostDeadband", new double[] {xVal, yVal, omgVal});
    Logger.recordOutput(
        "Teleop/FieldRelSpeeds",
        new double[] {
          targetSpeeds.vxMetersPerSecond,
          targetSpeeds.vyMetersPerSecond,
          targetSpeeds.omegaRadiansPerSecond
        });
    Logger.recordOutput(
        "Teleop/RobotRelSpeeds",
        new double[] {
          robotRelativeSpeeds.vxMetersPerSecond,
          robotRelativeSpeeds.vyMetersPerSecond,
          robotRelativeSpeeds.omegaRadiansPerSecond
        });
    Logger.recordOutput("Teleop/GyroLockActive", Math.abs(omgVal) <= 0.01);

    swerveDrive.runVelocity(robotRelativeSpeeds);
  }
}
