package com.marslib.mechanisms;

import org.littletonrobotics.junction.AutoLog;

/** Hardware abstraction interface for a linear translating mechanism (like an elevator). */
public interface LinearMechanismIO {
  @AutoLog
  public static class LinearMechanismIOInputs {
    public boolean hasHardwareConnected = true;
    public double positionMeters = 0.0;
    public double velocityMetersPerSec = 0.0;
    public double targetVelocityMetersPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  public default void updateInputs(LinearMechanismIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  /** Runs a low latency closed-loop profile on motor with external feedforward. */
  public default void setClosedLoopPosition(double positionMeters, double feedforwardVolts) {}

  public default void setCurrentLimit(double amps) {}

  public default void setBrakeMode(boolean enable) {}
}
