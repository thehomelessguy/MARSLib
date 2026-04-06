package com.marslib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Pure mathematics SLAM utility for calculating Projectile Kinematics (Shoot-On-The-Move). */
public class KinematicAiming {

  /**
   * Calculates a virtual target coordinate that perfectly offsets the robot's current driving
   * momentum. By aiming the robot (or turret) at this new virtual target rather than the real
   * target, the projectile will arc perfectly into the stationary target even while moving at high
   * speeds.
   *
   * @param currentRobotPose The true Odometeric pose of the robot.
   * @param robotVelocity The current discrete field-relative ChassisSpeeds (m/s).
   * @param realTarget The physical location of the stationary target (e.g. The Speaker).
   * @param projectileSpeedMps The empirically measured muzzle velocity of your launcher in meters /
   *     sec.
   * @return A virtual aiming Pose2d representing where you must point to hit the real target.
   */
  public static Pose2d calculateVirtualAimPoint(
      Pose2d currentRobotPose,
      ChassisSpeeds robotVelocity,
      Pose2d realTarget,
      double projectileSpeedMps) {

    // 1. Find straight-line distance to the real target
    double directDistanceMeters =
        currentRobotPose.getTranslation().getDistance(realTarget.getTranslation());

    // 2. Calculate the estimated Time of Flight (ToF) for the physical projectile
    double timeOfFlightSecs = directDistanceMeters / projectileSpeedMps;

    // 3. Extrapolate how far the robot's physical momentum will drag the projectile horizontally
    // over that time length
    double virtualX = realTarget.getX() - (robotVelocity.vxMetersPerSecond * timeOfFlightSecs);
    double virtualY = realTarget.getY() - (robotVelocity.vyMetersPerSecond * timeOfFlightSecs);

    // 4. Return the new coordinates to feed into the turret/drivetrain alignment system
    return new Pose2d(new Translation2d(virtualX, virtualY), realTarget.getRotation());
  }
}
