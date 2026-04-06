package com.marslib.swerve;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * Central constants for the MARSLib swerve drivetrain.
 *
 * <p>Students: Modify these values to match your physical robot. All distances are in meters, all
 * angles are in radians, and all speeds are in m/s or rad/s.
 */
public final class SwerveConstants {

  // ---- Chassis Geometry ----

  /** Distance between left and right module centers (meters). */
  public static final double TRACK_WIDTH_METERS = 0.6;

  /** Distance between front and rear module centers (meters). */
  public static final double WHEELBASE_METERS = 0.6;

  /** Positions of each swerve module relative to robot center (FL, FR, BL, BR). */
  public static final Translation2d[] MODULE_LOCATIONS =
      new Translation2d[] {
        new Translation2d(WHEELBASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0), // Front-Left
        new Translation2d(WHEELBASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0), // Front-Right
        new Translation2d(-WHEELBASE_METERS / 2.0, TRACK_WIDTH_METERS / 2.0), // Back-Left
        new Translation2d(-WHEELBASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0) // Back-Right
      };

  // ---- Wheel ----

  /** Radius of the swerve drive wheels (meters). Standard 2-inch colson = 0.0508m. */
  public static final double WHEEL_RADIUS_METERS = 0.0508;

  // ---- Drive Constraints ----

  /** Maximum achievable linear robot speed (m/s). */
  public static final double MAX_LINEAR_SPEED_MPS = 4.5;

  /** Maximum achievable angular robot speed (rad/s). */
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 2;

  // ---- Gear Ratios ----

  /** Drive motor gear ratio (motor rotations per output rotation). L1 = 6.75:1. */
  public static final double DRIVE_GEAR_RATIO = 6.75;

  /** Turn motor gear ratio (motor rotations per module rotation). */
  public static final double TURN_GEAR_RATIO = 21.4;

  // ---- Current Limits ----

  /** Nominal stator current limit for drive motors (amps). */
  public static final double DRIVE_STATOR_CURRENT_LIMIT = 80.0;

  /** Nominal stator current limit for turn motors (amps). */
  public static final double TURN_STATOR_CURRENT_LIMIT = 40.0;

  /** Minimum current limit during load shedding (amps). */
  public static final double MIN_LOAD_SHED_CURRENT = 20.0;

  // ---- Turn PID ----

  /** Proportional gain for the module turn voltage controller. */
  public static final double TURN_KP = 5.0;

  private SwerveConstants() {}
}
