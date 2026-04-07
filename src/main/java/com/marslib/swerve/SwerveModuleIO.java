package com.marslib.swerve;

import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction interface for a single swerve module (drive + turn motors).
 *
 * <p>Students: Implement this interface once for real hardware ({@code SwerveModuleIOTalonFX}) and
 * once for desktop physics simulation ({@code SwerveModuleIOSim}). The {@link SwerveModule}
 * subsystem is injected with one or the other and never knows the difference.
 */
public interface SwerveModuleIO {

  @AutoLog
  public static class SwerveModuleIOInputs {
    public boolean hasHardwareConnected = true;

    public double[] drivePositionsRad = new double[] {};
    public double[] turnPositionsRad = new double[] {};
    public double[] odometryTimestamps = new double[] {};

    public double driveVelocityRadPerSec = 0.0;
    public double turnVelocityRadPerSec = 0.0;

    public double driveAppliedVolts = 0.0;
    public double turnAppliedVolts = 0.0;

    public double driveCurrentAmps = 0.0;
    public double turnCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void setTurnVoltage(double volts) {}

  /** Sets the neutral mode of the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Sets the neutral mode of the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** Dynamically limits the drive motor current for load shedding. */
  public default void setCurrentLimit(double amps) {}
}
