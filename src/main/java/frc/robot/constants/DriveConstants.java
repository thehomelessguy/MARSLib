package frc.robot.constants;

public final class DriveConstants {
  public static final String CANBUS = "rio";

  // Front Left
  public static final int FL_DRIVE_ID = 7;
  public static final int FL_TURN_ID = 8;
  // Front Right
  public static final int FR_DRIVE_ID = 1;
  public static final int FR_TURN_ID = 2;
  // Back Left
  public static final int BL_DRIVE_ID = 6;
  public static final int BL_TURN_ID = 5;
  // Back Right
  public static final int BR_DRIVE_ID = 3;
  public static final int BR_TURN_ID = 4;

  public static final int PIGEON2_ID = 9;

  public static final double TELEOP_LINEAR_ACCEL_LIMIT = 15.0;
  public static final double TELEOP_OMEGA_ACCEL_LIMIT = Math.PI * 6.0;
  public static final double HEADING_KP = 5.0;

  public static final double TELEMETRY_HZ = 50.0;
  public static final double ODOMETRY_HZ = 250.0;
}
