package com.marslib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Advanced Shot-On-The-Move mathematical solver ingested from Team 254 (2024). This utility
 * calculates exact trajectory kinematics, solving the quadratic equation for time-of-flight while
 * applying gravity and lift compensation.
 */
public class EliteShooterMath {

  /**
   * Data class for holding calculated Elite Shooter parameters. Fully compatible with AdvantageKit
   * logging.
   */
  public static class EliteShooterSetpoint {
    public double robotAimYawRadians;
    public double chassisAngularFeedforward;
    public double hoodRadians;
    public double hoodFeedforward;
    public double launchSpeedMetersPerSec;
    public boolean isValid;
  }

  /**
   * Mathematically solves the exact shot state needed to hit a 3D target given current robot speeds
   * and constraints.
   *
   * @param robotPose Current field-relative robot pose
   * @param fieldRelativeSpeeds Current field-relative speeds of the chassis
   * @param targetTranslation Exact 3D field coordinates of the target
   * @param releaseHeightZ Height of the robot's shooter mechanism from the floor
   * @param nominalShotSpeedMetersPerSec Base shot velocity output limit
   * @param gravity Gravity constant (typically -9.81)
   * @param liftCoefficient Aerodynamic lift coefficient of the game piece
   * @return Computed EliteShooterSetpoint with exact angles and feedforwards
   */
  public static EliteShooterSetpoint calculateShotOnTheMove(
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds,
      Translation3d targetTranslation,
      double releaseHeightZ,
      double nominalShotSpeedMetersPerSec,
      double gravity,
      double liftCoefficient) {

    EliteShooterSetpoint setpoint = new EliteShooterSetpoint();

    Translation3d robotToTarget =
        targetTranslation.minus(
            new Translation3d(robotPose.getX(), robotPose.getY(), releaseHeightZ));

    double vShot = nominalShotSpeedMetersPerSec;

    // Solve quadratic equation to obtain time of flight of game piece.
    // a = vx^2 + vy^2 - vShot^2
    // b = -2 * ((tx - rx) * vx + (ty - ry) * vy)
    // c = (tx - rx)^2 + (ty - ry)^2 + dz^2
    double a =
        fieldRelativeSpeeds.vxMetersPerSecond * fieldRelativeSpeeds.vxMetersPerSecond
            + fieldRelativeSpeeds.vyMetersPerSecond * fieldRelativeSpeeds.vyMetersPerSecond
            - vShot * vShot;

    if (Math.abs(a) < 1e-6) {
      // Cheat slightly to avoid division by zero / non-quadratic states
      vShot = 1.01 * vShot;
      a =
          fieldRelativeSpeeds.vxMetersPerSecond * fieldRelativeSpeeds.vxMetersPerSecond
              + fieldRelativeSpeeds.vyMetersPerSecond * fieldRelativeSpeeds.vyMetersPerSecond
              - vShot * vShot;
    }

    double b =
        -2.0
            * (robotToTarget.getX() * fieldRelativeSpeeds.vxMetersPerSecond
                + robotToTarget.getY() * fieldRelativeSpeeds.vyMetersPerSecond);

    double c =
        robotToTarget.getX() * robotToTarget.getX()
            + robotToTarget.getY() * robotToTarget.getY()
            + robotToTarget.getZ() * robotToTarget.getZ();

    double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
      discriminant = 0.0;
    }

    // Solve for time of flight (t)
    double t = (-b - Math.sqrt(discriminant)) / (2.0 * a);

    if (t <= 0) {
      setpoint.isValid = false;
      return setpoint;
    }

    Translation3d virtualShot =
        new Translation3d(
            (robotToTarget.getX() - fieldRelativeSpeeds.vxMetersPerSecond * t) / t,
            (robotToTarget.getY() - fieldRelativeSpeeds.vyMetersPerSecond * t) / t,
            (robotToTarget.getZ() / t));

    Rotation2d virtualTargetRotation = new Rotation2d(virtualShot.getX(), virtualShot.getY());
    double xyVel =
        Math.sqrt(
            virtualShot.getX() * virtualShot.getX() + virtualShot.getY() * virtualShot.getY());

    // Apply gravity and lift compensation
    double drop = 0.5 * t * t * gravity;
    drop += 0.5 * liftCoefficient * c;

    double pitchAngleRads = Math.atan2((robotToTarget.getZ() - drop) / t, xyVel);
    double adjustedVShot =
        Math.sqrt(
            (robotToTarget.getZ() - drop) * (robotToTarget.getZ() - drop) / (t * t)
                + xyVel * xyVel);

    // Compute Chassis Aim and Feedforward
    double distanceToTarget =
        Math.sqrt(
            robotToTarget.getX() * robotToTarget.getX()
                + robotToTarget.getY() * robotToTarget.getY());

    double chassisAngularFF =
        (robotToTarget.getY() * fieldRelativeSpeeds.vxMetersPerSecond
                - robotToTarget.getX() * fieldRelativeSpeeds.vyMetersPerSecond)
            / (distanceToTarget * distanceToTarget);

    Translation2d targetToRobotFrame =
        new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond)
            .rotateBy(virtualTargetRotation);

    double hoodFF =
        targetToRobotFrame.getX()
            * -robotToTarget.getZ()
            / (distanceToTarget * distanceToTarget + robotToTarget.getZ() * robotToTarget.getZ());

    setpoint.robotAimYawRadians = virtualTargetRotation.getRadians();
    setpoint.chassisAngularFeedforward = chassisAngularFF;
    setpoint.hoodRadians = pitchAngleRads;
    setpoint.hoodFeedforward = hoodFF;
    setpoint.launchSpeedMetersPerSec = adjustedVShot;
    setpoint.isValid = true;

    return setpoint;
  }
}
