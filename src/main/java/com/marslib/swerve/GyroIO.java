package com.marslib.swerve;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware interface for the robot's central gyroscope (IMU). Provides yaw position and velocity
 * for pose estimation and field-centric drive.
 */
public interface GyroIO {

  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public double yawPositionRad = 0.0;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawPositions = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GyroIOInputs inputs) {}
}
