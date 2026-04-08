package com.marslib.util;

import com.marslib.swerve.SwerveDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Shot Setup Utility for computing Shot-On-The-Move (SOTM) parameters. Calculates predictive aiming
 * lookahead, time-of-flight convergence, and angular velocity feedforwards, while being
 * game-agnostic and robot-agnostic.
 */
public class ShotSetup {

  private final double phaseDelay;
  private final double maxCowlPosition;
  private final double maxFlywheelSpeedRPM;
  private final double recomputeThreshold;
  private final int convergenceIters;
  private final double convergenceEpsilon;
  private final double minTof;
  private final double maxTof;

  private final Transform2d robotToShooterTransform;
  private final Rotation2d operatorForwardDirection;

  private final InterpolatingTreeMap<Double, ShotInfo> shotMap;
  private final InterpolatingTreeMap<Double, Double> tofMap;

  private SOTMInfo cachedSOTMInfo = null;
  private double lastTargetDist = -1;

  /** Data class holding raw shooter configuration details. */
  public static class Shot {
    public final double shooterRPM;

    public Shot(double shooterRPM) {
      this.shooterRPM = shooterRPM;
    }
  }

  /** Data class holding complete shooter mechanism states. */
  public static class ShotInfo {
    public final Shot shot;
    public final double cowlPosition;

    public ShotInfo(Shot shot, double cowlPosition) {
      this.shot = shot;
      this.cowlPosition = cowlPosition;
    }
  }

  /** Data class holding converged SOTM calculations. */
  public static class SOTMInfo {
    public final ShotInfo shotInfo;
    public final Rotation2d virtualTargetAngle;
    public final double angularVelocityRadPerSec;

    public SOTMInfo(
        ShotInfo shotInfo, Rotation2d virtualTargetAngle, double angularVelocityRadPerSec) {
      this.shotInfo = shotInfo;
      this.virtualTargetAngle = virtualTargetAngle;
      this.angularVelocityRadPerSec = angularVelocityRadPerSec;
    }
  }

  /** Constructs the SOTM calculation utility with physical constraints. */
  public ShotSetup(
      double phaseDelay,
      double maxCowlPosition,
      double maxFlywheelSpeedRPM,
      double recomputeThreshold,
      int convergenceIters,
      double convergenceEpsilon,
      double minTof,
      double maxTof,
      Transform2d robotToShooterTransform,
      Rotation2d operatorForwardDirection) {
    this.phaseDelay = phaseDelay;
    this.maxCowlPosition = maxCowlPosition;
    this.maxFlywheelSpeedRPM = maxFlywheelSpeedRPM;
    this.recomputeThreshold = recomputeThreshold;
    this.convergenceIters = convergenceIters;
    this.convergenceEpsilon = convergenceEpsilon;
    this.minTof = minTof;
    this.maxTof = maxTof;
    this.robotToShooterTransform = robotToShooterTransform;
    this.operatorForwardDirection = operatorForwardDirection;

    this.shotMap =
        new InterpolatingTreeMap<>(
            (s, e, q) -> InverseInterpolator.forDouble().inverseInterpolate(s, e, q),
            (s, e, t) ->
                new ShotInfo(
                    new Shot(
                        Interpolator.forDouble()
                            .interpolate(s.shot.shooterRPM, e.shot.shooterRPM, t)),
                    Interpolator.forDouble().interpolate(s.cowlPosition, e.cowlPosition, t)));

    this.tofMap =
        new InterpolatingTreeMap<>(
            (s, e, q) -> InverseInterpolator.forDouble().inverseInterpolate(s, e, q),
            (s, e, t) -> Interpolator.forDouble().interpolate(s, e, t));
  }

  /** Register a shot map entry correlating distance to RPM and Cowl Pivot. */
  public void addShotMapEntry(double distanceMeters, double shooterRPM, double cowlPosition) {
    shotMap.put(distanceMeters, new ShotInfo(new Shot(shooterRPM), cowlPosition));
  }

  /** Register a time of flight entry correlating distance to time of flight in seconds. */
  public void addTofMapEntry(double distanceMeters, double timeOfFlightSec) {
    tofMap.put(distanceMeters, timeOfFlightSec);
  }

  /** Retrieves static shot settings uncompensated for motion. */
  public ShotInfo getStaticShotInfo(double targetDistMeters) {
    ShotInfo raw = shotMap.get(targetDistMeters);
    if (raw == null) {
      return new ShotInfo(new Shot(0), 0);
    }
    return clamp(raw);
  }

  /**
   * Retrieves dynamically predicted Shot-On-The-Move settings based on current kinematics and a
   * given target. Caches outputs spatially to save iterative convergence cycles if the robot hasn't
   * moved significantly.
   */
  public SOTMInfo getSOTMInfo(SwerveDrive swerve, Translation2d target) {
    double currentDist = target.getDistance(swerve.getPose().getTranslation());

    // Cache Hit Check
    if (cachedSOTMInfo != null && Math.abs(currentDist - lastTargetDist) < recomputeThreshold) {
      return cachedSOTMInfo;
    }

    lastTargetDist = currentDist;
    cachedSOTMInfo = computeSOTM(swerve, target);
    return cachedSOTMInfo;
  }

  /** Computes the actual SOTM iterative logic (originally derived from MXIX). */
  private SOTMInfo computeSOTM(SwerveDrive swerve, Translation2d target) {
    Pose2d robotPose = swerve.getPose();
    ChassisSpeeds robotSpeeds = swerve.getChassisSpeeds();

    // 1. Phase-delay compensated pose
    Pose2d futureRobotPose =
        robotPose.exp(
            new Twist2d(
                robotSpeeds.vxMetersPerSecond * phaseDelay,
                robotSpeeds.vyMetersPerSecond * phaseDelay,
                robotSpeeds.omegaRadiansPerSecond * phaseDelay));

    // Offset the pose to the mathematical center of the shooter
    Pose2d shotPose =
        futureRobotPose.plus(
            new Transform2d(
                robotToShooterTransform.getTranslation(), robotToShooterTransform.getRotation()));

    // 2. Field-relative shooter velocity
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, robotPose.getRotation());

    double cosH = robotPose.getRotation().getCos();
    double sinH = robotPose.getRotation().getSin();
    double offsetX = robotToShooterTransform.getX() * cosH - robotToShooterTransform.getY() * sinH;
    double offsetY = robotToShooterTransform.getX() * sinH + robotToShooterTransform.getY() * cosH;
    double omega = fieldSpeeds.omegaRadiansPerSecond;

    // Effective translational velocity at the shooter node
    double velX = fieldSpeeds.vxMetersPerSecond + -offsetY * omega;
    double velY = fieldSpeeds.vyMetersPerSecond + offsetX * omega;

    // 3. Iterative convergence for Time of Flight
    Pose2d lookaheadPose = shotPose;
    double lookaheadDist = target.getDistance(shotPose.getTranslation());

    for (int i = 0; i < convergenceIters; i++) {
      Double tofNullable = tofMap.get(lookaheadDist);
      double tofRaw = (tofNullable != null) ? tofNullable : minTof;
      double tof = MathUtil.clamp(tofRaw, minTof, maxTof);

      lookaheadPose =
          new Pose2d(
              shotPose.getTranslation().plus(new Translation2d(velX * tof, velY * tof)),
              shotPose.getRotation());

      double newDist = target.getDistance(lookaheadPose.getTranslation());
      if (Math.abs(newDist - lookaheadDist) < convergenceEpsilon) {
        break;
      }
      lookaheadDist = newDist;
    }

    // 4. Fallback lookup for shot parameters
    ShotInfo rawShot = shotMap.get(lookaheadDist);
    if (rawShot == null) {
      rawShot = new ShotInfo(new Shot(0), 0);
    }
    ShotInfo clamped = clamp(rawShot);

    // 5. Angular velocity feedforward
    double angularVelocity = 0;
    if (lookaheadDist > 0.1) {
      double rx = target.getX() - lookaheadPose.getX();
      double ry = target.getY() - lookaheadPose.getY();
      angularVelocity = (ry * velX - rx * velY) / (lookaheadDist * lookaheadDist);
    }

    // 6. Final aiming rotation angle
    Rotation2d directionFieldRel = target.minus(lookaheadPose.getTranslation()).getAngle();
    Rotation2d directionOperatorRel = directionFieldRel.rotateBy(operatorForwardDirection);

    return new SOTMInfo(clamped, directionOperatorRel, angularVelocity);
  }

  private ShotInfo clamp(ShotInfo raw) {
    return new ShotInfo(
        new Shot(MathUtil.clamp(raw.shot.shooterRPM, 0, maxFlywheelSpeedRPM)),
        MathUtil.clamp(raw.cowlPosition, 0, maxCowlPosition));
  }
}
