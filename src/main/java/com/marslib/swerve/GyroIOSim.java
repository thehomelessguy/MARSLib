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

  /**
   * Called by SwerveDrive to update the simulated gyro yaw from kinematics each loop.
   *
   * @param omegaRadPerSec The measured rotational velocity of the chassis (rad/s).
   * @param dtSeconds The time step of the control loop (seconds).
   */
  public void updateYawVelocity(double omegaRadPerSec, double dtSeconds) {
    yawVelocityRadPerSec = omegaRadPerSec;
    yawPositionRad += omegaRadPerSec * dtSeconds;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPositionRad = yawPositionRad;
    inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
  }
}
