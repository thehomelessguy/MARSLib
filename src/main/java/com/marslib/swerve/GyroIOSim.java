package com.marslib.swerve;

/**
 * Simulated gyro IO layer that derives yaw from chassis kinematics.
 *
 * <p>Students: In simulation, there is no physical IMU. Instead, the SwerveDrive subsystem
 * calculates the robot's rotational velocity from measured module states and feeds it into this
 * class, which integrates over time to track the heading.
 */
public class GyroIOSim implements GyroIO {
  private double yawPositionRad = 0.0;
  private double yawVelocityRadPerSec = 0.0;
  private final java.util.Random random = new java.util.Random();
  private double accumulatedDriftRad = 0.0;

  /**
   * Called by SwerveDrive to update the simulated gyro yaw from kinematics each loop.
   *
   * @param omegaRadPerSec The measured rotational velocity of the chassis (rad/s).
   * @param dtSeconds The time step of the control loop (seconds).
   */
  public void updateYawVelocity(double omegaRadPerSec, double dtSeconds) {
    yawVelocityRadPerSec = omegaRadPerSec;
    yawPositionRad += omegaRadPerSec * dtSeconds;

    if (frc.robot.constants.SimulationConstants.ENABLE_GYRO_NOISE) {
      accumulatedDriftRad +=
          Math.toRadians(frc.robot.constants.SimulationConstants.GYRO_DRIFT_DEG_PER_SEC)
              * dtSeconds;
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;

    double noiseRads = 0.0;
    if (frc.robot.constants.SimulationConstants.ENABLE_GYRO_NOISE) {
      noiseRads =
          random.nextGaussian()
              * frc.robot.constants.SimulationConstants.GYRO_NOISE_SCALAR
              * (1.0 + Math.abs(yawVelocityRadPerSec));
    }

    double noisyYaw = yawPositionRad + accumulatedDriftRad + noiseRads;

    inputs.yawPositionRad = noisyYaw;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
    // Provide per-frame yaw data so the pose estimator drain loop uses interpolated
    // yaw instead of a single scalar — prevents rotational aliasing during fast turns.
    inputs.odometryYawPositions = new double[] {noisyYaw};
  }
}
