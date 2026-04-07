// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "SIM_MODE" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode SIM_MODE = Mode.SIM;
  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;
  public static final double LOOP_PERIOD_SECS = 0.01;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DriveConstants {
    public static final String CANBUS = "rio";

    // Front Left
    public static final int FL_DRIVE_ID = 1;
    public static final int FL_TURN_ID = 2;
    // Front Right
    public static final int FR_DRIVE_ID = 3;
    public static final int FR_TURN_ID = 4;
    // Back Left
    public static final int BL_DRIVE_ID = 5;
    public static final int BL_TURN_ID = 6;
    // Back Right
    public static final int BR_DRIVE_ID = 7;
    public static final int BR_TURN_ID = 8;

    public static final int PIGEON2_ID = 9;

    public static final double TELEOP_LINEAR_ACCEL_LIMIT = 15.0;
    public static final double TELEOP_OMEGA_ACCEL_LIMIT = Math.PI * 6.0;
    public static final double HEADING_KP = 5.0;

    public static final double TELEMETRY_HZ = 50.0;
    public static final double ODOMETRY_HZ = 250.0;
  }

  public static final class ElevatorConstants {
    public static final int MOTOR_ID = 20;
    public static final String CANBUS = "rio";
    public static final double GEAR_RATIO = 10.0;
    public static final double SPOOL_DIAMETER_METERS = 0.05;
    public static final double SIM_MASS_KG = 5.0;
    public static final boolean INVERTED = false;

    // Load Shedding Bound Limits
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double CRITICAL_VOLTAGE = 7.0;
    public static final double MAX_CURRENT_AMPS = 60.0;
    public static final double MIN_CURRENT_AMPS = 20.0;
  }

  public static final class ArmConstants {
    public static final int MOTOR_ID = 21;
    public static final String CANBUS = "rio";
    public static final double GEAR_RATIO = 50.0;
    public static final double SIM_MOI = 1.0;
    public static final double SIM_LENGTH = 0.6;
    public static final boolean INVERTED = false;

    // Load Shedding Bound Limits
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double CRITICAL_VOLTAGE = 7.0;
    public static final double MAX_CURRENT_AMPS = 60.0;
    public static final double MIN_CURRENT_AMPS = 20.0;
  }

  public static final class IntakeConstants {
    public static final int MOTOR_ID = 22;
    public static final String CANBUS = "rio";
  }

  public static final class ShooterConstants {
    public static final int MOTOR_ID = 23;
    public static final String CANBUS = "rio";
    public static final double PROJECTILE_SPEED_MPS = 15.0;
  }

  public static final class SuperstructureConstants {
    public static final double SAFE_ELEVATOR_HEIGHT_METERS_MIN = 0.6;
    public static final double SAFE_ELEVATOR_HEIGHT_METERS_MAX_STOW = 0.5;
    public static final double SAFE_ARM_ANGLE_RAD_MAX_STOW = 0.2;
    public static final double SAFE_ARM_ANGLE_RAD_MIN_EXTEND = 0.3;

    public static final double INTAKE_ELEVATOR_HEIGHT = 0.2;
    public static final double INTAKE_ARM_ANGLE = Math.PI / 4;

    public static final double SCORE_HIGH_ELEVATOR_HEIGHT = 1.5;
    public static final double SCORE_HIGH_ARM_ANGLE = Math.PI / 2;
  }

  public static final class LEDConstants {
    public static final int PWM_PORT = 0;
    public static final int LENGTH = 60;
  }

  public static final class PowerConstants {
    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final double WARNING_VOLTAGE = 8.0;
    public static final double CRITICAL_VOLTAGE = 7.0;
  }

  public static final class AutoConstants {
    public static final double MAX_VELOCITY_MPS = 3.0;
    public static final double MAX_ACCELERATION_MPS2 = 2.0;
    public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC2 = Math.PI / 2.0;
    public static final String GHOST_MACRO_FILE_PATH =
        edu.wpi.first.wpilibj.Filesystem.getOperatingDirectory().getAbsolutePath()
            + "/ghost_macro.csv";
    public static final double ALIGN_TRANSLATION_KP = 5.0;
    public static final double ALIGN_THETA_KP = 5.0;

    public static final double ALIGN_TRANSLATION_MAX_VELOCITY_MPS = 3.0;
    public static final double ALIGN_TRANSLATION_MAX_ACCEL_MPS2 = 3.0;
    public static final double ALIGN_ROTATION_MAX_VELOCITY_RAD_PER_SEC = Math.PI * 2;
    public static final double ALIGN_ROTATION_MAX_ACCEL_RAD_PER_SEC2 = Math.PI * 4;

    public static final double ALIGN_TRANSLATION_TOLERANCE_METERS = 0.05;
    public static final double ALIGN_ROTATION_TOLERANCE_RAD = 0.05;
  }

  public static final class SimulationConstants {
    public static final boolean ENABLE_VISION_OCCLUSION = false;
    public static final double VISION_OCCLUSION_DROP_PROBABILITY = 0.05;

    public static final boolean ENABLE_GYRO_NOISE = false;
    public static final double GYRO_DRIFT_DEG_PER_SEC = 0.05;
    public static final double GYRO_NOISE_SCALAR = 0.005;

    public static final boolean ENABLE_CAN_STARVATION = false;
    public static final double CAN_STARVATION_PROBABILITY = 0.02;
    public static final int CAN_STARVATION_DELAY_MS = 2;
  }

  public static final class VisionConstants {
    /** Base linear standard deviation for single-tag AprilTag poses. */
    public static final double TAG_STD_BASE = 0.05;

    /** Maximum acceptable ambiguity ratio for single-tag observations. */
    public static final double MAX_AMBIGUITY = 0.2;

    /** Maximum acceptable Z-height offset (meters) to reject 'flying robot' hallucinations. */
    public static final double MAX_Z_HEIGHT = 0.5;

    /** Multiplier applied to linear StdDev when multiple tags are simultaneously visible. */
    public static final double MULTI_TAG_STD_MULTIPLIER = 0.1;

    /** Multiplier to convert linear StdDev to angular StdDev for the pose estimator. */
    public static final double ANGULAR_STD_MULTIPLIER = 2.0;

    /** Static standard deviation for VIO SLAM measurements (meters and radians). */
    public static final double SLAM_STD_DEV = 0.01;
  }

  public static final class OperatorConstants {
    public static final int PULSE_DURATION_LOOPS = 25; // 500ms
    public static final int PULSE_INTERVAL_LOOPS = 25; // 500ms
  }

  public static final class FieldConstants {
    /** Standard FRC field length (meters). */
    public static final double FIELD_LENGTH_METERS = 16.541;

    /** Standard FRC field width (meters). */
    public static final double FIELD_WIDTH_METERS = 8.069;

    /** Thickness of simulated field boundary walls (meters). */
    public static final double WALL_THICKNESS_METERS = 1.0;

    /** Approximate side length of the hub/reef structures (meters). */
    public static final double HUB_SIZE_METERS = 1.2;

    /** Height above the field surface at which game pieces rest in 3D visualization (meters). */
    public static final double GAME_PIECE_REST_HEIGHT_METERS = 0.1;

    /** Radius of a standard game piece for collision modeling (meters). */
    public static final double GAME_PIECE_RADIUS_METERS = 0.1;

    /** Intake collection radius (meters) for swallowing pieces in simulation. */
    public static final double INTAKE_COLLECTION_RADIUS_METERS = 1.0;

    /** Mass of a standard game piece (kg). */
    public static final double GAME_PIECE_MASS_KG = 0.2;

    /** Coulomb friction coefficient for field boundary walls and obstacles. */
    public static final double WALL_FRICTION = 0.2;

    /** Coefficient of restitution (bounciness) for field boundary walls and obstacles. */
    public static final double WALL_RESTITUTION = 0.1;

    public static final edu.wpi.first.math.geometry.Translation2d RED_HUB_POS =
        new edu.wpi.first.math.geometry.Translation2d(11.91, 4.03);
    public static final edu.wpi.first.math.geometry.Translation2d BLUE_HUB_POS =
        new edu.wpi.first.math.geometry.Translation2d(4.62, 4.03);
  }
}
