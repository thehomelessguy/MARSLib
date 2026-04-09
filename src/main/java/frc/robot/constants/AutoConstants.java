package frc.robot.constants;

import com.marslib.mechanisms.*;

public final class AutoConstants {
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
