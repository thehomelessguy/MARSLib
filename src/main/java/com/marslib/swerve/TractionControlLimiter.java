package com.marslib.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Advanced 2D Slew Rate Limiter to prevent carpet slippage and Odometry drift.
 *
 * <p>Standard WPILib SlewRateLimiters operate independently on X and Y axes. If both max out, the
 * robot accelerates diagonally at 1.41x the limit, instantly breaking carpet friction. This class
 * ensures the total 2D acceleration vector never exceeds the maximum static friction coefficient of
 * the FRC carpet (usually ~1.1g or 10.78 m/s^2).
 */
public class TractionControlLimiter {
  private final double maxAccelMetersPerSecSq;
  private Translation2d lastVelocity = new Translation2d();
  private double lastTime = Timer.getFPGATimestamp();

  /**
   * Initializes the 2D Traction Control limit.
   *
   * @param maxAccelMetersPerSecSq Maximum physical acceleration before tire slip occurs.
   */
  public TractionControlLimiter(double maxAccelMetersPerSecSq) {
    this.maxAccelMetersPerSecSq = maxAccelMetersPerSecSq;
  }

  /**
   * Calculates the max achievable velocity vector without slipping tires.
   *
   * @param targetVelocity Target requested X/Y velocity vector.
   * @return Safe velocity vector to apply to the ChassisSpeeds.
   */
  public Translation2d calculate(Translation2d targetVelocity) {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    // Prevent divide-by-zero on first loop
    if (dt <= 0.0) return lastVelocity;

    // Calculate requested velocity change
    Translation2d deltaV = targetVelocity.minus(lastVelocity);

    // Total acceleration required to achieve this change
    double currentAccel = deltaV.getNorm() / dt;

    if (currentAccel > maxAccelMetersPerSecSq) {
      // Scale down the change to exactly match peak acceleration
      double scalar = maxAccelMetersPerSecSq / currentAccel;
      deltaV = deltaV.times(scalar);
    }

    lastVelocity = lastVelocity.plus(deltaV);

    // Ensure we don't float slightly off 0.0 due to dt calculation margins
    if (targetVelocity.getNorm() == 0.0 && lastVelocity.getNorm() < 0.05) {
      lastVelocity = new Translation2d();
    }

    return lastVelocity;
  }
}
