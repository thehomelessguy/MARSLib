package frc.robot.constants;

import com.marslib.util.LoggedTunableNumber;

public final class VisionConstants {
  public static final String CAMERA_0_NAME = "limelight-shooter";
  public static final String CAMERA_1_NAME = "limelight-back";

  /** Base linear standard deviation for single-tag AprilTag poses. */
  public static final LoggedTunableNumber TAG_STD_BASE =
      new LoggedTunableNumber("Vision/TAG_STD_BASE", 0.05);

  /** Maximum acceptable ambiguity ratio for single-tag observations. */
  public static final LoggedTunableNumber MAX_AMBIGUITY =
      new LoggedTunableNumber("Vision/MAX_AMBIGUITY", 0.2);

  /** Maximum acceptable Z-height offset (meters) to reject 'flying robot' hallucinations. */
  public static final LoggedTunableNumber MAX_Z_HEIGHT =
      new LoggedTunableNumber("Vision/MAX_Z_HEIGHT", 0.5);

  /** Multiplier applied to linear StdDev when multiple tags are simultaneously visible. */
  public static final LoggedTunableNumber MULTI_TAG_STD_MULTIPLIER =
      new LoggedTunableNumber("Vision/MULTI_TAG_STD_MULTIPLIER", 0.1);

  /** Multiplier to convert linear StdDev to angular StdDev for the pose estimator. */
  public static final LoggedTunableNumber ANGULAR_STD_MULTIPLIER =
      new LoggedTunableNumber("Vision/ANGULAR_STD_MULTIPLIER", 2.0);

  /** Static standard deviation for VIO SLAM measurements (meters and radians). */
  public static final LoggedTunableNumber SLAM_STD_DEV =
      new LoggedTunableNumber("Vision/SLAM_STD_DEV", 0.01);

  /** Static standard deviation for VIO SLAM angular measurements (radians). */
  public static final LoggedTunableNumber SLAM_ANGULAR_STD_DEV =
      new LoggedTunableNumber("Vision/SLAM_ANGULAR_STD_DEV", 0.5);

  private VisionConstants() {}
}
