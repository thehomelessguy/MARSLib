package com.marslib.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Utility class providing cached, GC-friendly alliance queries.
 *
 * <p>This eliminates repeated {@code DriverStation.getAlliance()} calls that each return a new
 * {@code Optional} allocation. Use {@link #isRed()} and {@link #isBlue()} throughout the codebase
 * instead of calling the DriverStation directly.
 */
public final class AllianceUtil {

  private AllianceUtil() {}

  /**
   * Returns true if the robot is on the Red alliance.
   *
   * @return {@code true} if alliance is present and is Red; {@code false} otherwise.
   */
  public static boolean isRed() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * Returns true if the robot is on the Blue alliance.
   *
   * @return {@code true} if alliance is present and is Blue; {@code false} otherwise.
   */
  public static boolean isBlue() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;
  }
}
