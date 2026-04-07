package frc.robot;

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

  // ---- Physics ----

  /** Mass of the robot inclusive of battery and bumpers (kg). 140lbs. */
  public static final double ROBOT_MASS_KG = 63.5029;

  /**
   * Outer dimensions of the robot with bumpers, used for bounding boxes and MoI approximation (m)
   */
  public static final double BUMPER_LENGTH_METERS = 0.8;

  public static final double BUMPER_WIDTH_METERS = 0.8;

  /** Approximate moment of inertia of the robot based on a solid block model (kg*m^2) */
  public static final double ROBOT_MOI_KG_M2 =
      (ROBOT_MASS_KG / 12.0)
          * (Math.pow(BUMPER_LENGTH_METERS, 2) + Math.pow(BUMPER_WIDTH_METERS, 2));

  /** Static coefficient of friction of the tread against standard FRC carpet. */
  public static final double WHEEL_COF_STATIC = 1.2;

  /** Kinetic coefficient of friction once the wheel breaks traction (wheel slip/burnout). */
  public static final double WHEEL_COF_KINETIC = 0.8;

  /**
   * Slip velocity threshold (m/s) at which static friction transitions to kinetic.
   *
   * <p>Below this velocity, wheel grip uses {@link #WHEEL_COF_STATIC}. Above it, grip drops to
   * {@link #WHEEL_COF_KINETIC}, modeling the Stribeck/Coulomb friction curve.
   */
  public static final double SLIP_TRANSITION_VELOCITY_MPS = 0.05;

  /**
   * Lateral slip relaxation length (m/s). Controls the viscous ramp region for sideways tire
   * friction to prevent numerical jitter at low lateral velocities.
   */
  public static final double LATERAL_SLIP_RELAXATION = 0.1;

  /**
   * Effective rotational inertia of a single driven wheel assembly including the Kraken X60 rotor
   * reflected through the {@link #DRIVE_GEAR_RATIO} (kg·m²).
   */
  public static final double WHEEL_MOI_KG_M2 = 0.002;

  /** Standard gravitational acceleration (m/s²). */
  public static final double GRAVITY_M_PER_S2 = 9.81;

  // ---- Simulation Collision ----

  /** Coefficient of friction for robot bumpers scraping against field walls. */
  public static final double WALL_FRICTION = 0.5;

  /** Coefficient of restitution (bounciness) for robot-to-wall collisions. */
  public static final double WALL_RESTITUTION = 0.1;

  // ---- Wheel ----

  /** Radius of the swerve drive wheels (meters). Standard 2-inch colson = 0.0508m. */
  public static final double WHEEL_RADIUS_METERS = 0.0508;

  // ---- Drive Constraints ----

  /** Maximum achievable linear robot speed (m/s). */
  public static final double MAX_LINEAR_SPEED_MPS = 4.5;

  /** Maximum achievable angular robot speed (rad/s). */
  public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = Math.PI * 2;

  /**
   * Nominal battery voltage used for drive feedforward normalization (V).
   *
   * <p>This is the true battery nominal (12.0V), NOT the load-shedding trigger threshold ({@link
   * frc.robot.Constants.PowerConstants#NOMINAL_VOLTAGE}). Used in {@link
   * com.marslib.swerve.SwerveModule#setDesiredState} to scale desired speed into voltage.
   */
  public static final double NOMINAL_BATTERY_VOLTAGE = 12.0;

  // ---- Gear Ratios ----

  /** Drive motor gear ratio (motor rotations per output rotation). L1 = 6.75:1. */
  public static final double DRIVE_GEAR_RATIO = 6.75;

  /** Turn motor gear ratio (motor rotations per module rotation). */
  public static final double TURN_GEAR_RATIO = 21.4;

  // ---- Current Limits ----

  /** Nominal stator current limit for drive motors (amps). */
  public static final double DRIVE_STATOR_CURRENT_LIMIT = 60.0;

  /** Nominal stator current limit for turn motors (amps). */
  public static final double TURN_STATOR_CURRENT_LIMIT = 40.0;

  /** Minimum current limit during load shedding (amps). */
  public static final double MIN_LOAD_SHED_CURRENT = 20.0;

  // ---- Turn PID ----

  /** Proportional gain for the module turn voltage controller. */
  public static final double TURN_KP = 5.0;

  private SwerveConstants() {}
}
