package frc.robot.constants;

public final class SimulationConstants {
  public static final boolean ENABLE_VISION_OCCLUSION = false;
  public static final double VISION_OCCLUSION_DROP_PROBABILITY = 0.05;

  public static final boolean ENABLE_GYRO_NOISE = false;
  public static final double GYRO_DRIFT_DEG_PER_SEC = 0.05;
  public static final double GYRO_NOISE_SCALAR = 0.005;

  public static final boolean ENABLE_CAN_STARVATION = false;
  public static final double CAN_STARVATION_PROBABILITY = 0.02;
  public static final int CAN_STARVATION_DELAY_MS = 2;
}
