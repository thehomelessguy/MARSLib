package com.marslib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/**
 * Represents a singular Swerve Module (e.g. Front-Left, Front-Right). This class abstracts the
 * actual hardware implementation (TalonFX vs Sim) via the {@link SwerveModuleIO} layer.
 *
 * <p>Students: This class handles the math converting raw Radians from the IO layer into standard
 * WPILib Meters and Meters/Second parameters for the PoseEstimator.
 */
public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
  private final int index;

  /**
   * Constructs a generic Swerve Module boundary structure.
   *
   * @param index The ID/Index (0=FL, 1=FR, 2=BL, 3=BR) for structural AdvantageKit logging keys.
   * @param io The implementation-specific IO layer (TalonFX or Sim).
   */
  public SwerveModule(int index, SwerveModuleIO io) {
    this.index = index;
    this.io = io;
  }

  /**
   * Polls the underlying IO interface for fresh odometry states and processes inputs natively into
   * AdvantageKit log streams.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("SwerveDrive/Module" + index, inputs);
  }

  /**
   * Iterates over the high-frequency positional arrays produced by internal Phoenix 6 Odometry
   * Threads to generate intermediate WPILib poses across a single 20ms delta window.
   *
   * @return Array of all physical module locations over the delta window in abstract WPILib
   *     kinematics terms.
   */
  public SwerveModulePosition[] getPositionDeltas() {
    int length = inputs.drivePositionsRad.length;
    if (length == 0) {
      return new SwerveModulePosition[] {new SwerveModulePosition()};
    }

    SwerveModulePosition[] positions = new SwerveModulePosition[length];
    // Wheel radius defaults to 2 inches (0.0508) for MARSLib standard
    final double wheelRadiusMeters = 0.0508;

    for (int i = 0; i < length; i++) {
      double distanceMeters = inputs.drivePositionsRad[i] * wheelRadiusMeters;
      Rotation2d angle = new Rotation2d(inputs.turnPositionsRad[i]);
      positions[i] = new SwerveModulePosition(distanceMeters, angle);
    }
    return positions;
  }

  /**
   * Returns the most recent physical location recorded by the drive motors.
   *
   * @return A singular {@link SwerveModulePosition} bounding distance traveled and heading.
   */
  public SwerveModulePosition getLatestPosition() {
    SwerveModulePosition[] deltas = getPositionDeltas();
    return deltas[deltas.length - 1];
  }

  /**
   * Translates drive wheel RPS and turn module radians into WPILib Velocity metrics.
   *
   * @return A {@link SwerveModuleState} tracking linear velocity (m/s) and angular heading.
   */
  public SwerveModuleState getLatestState() {
    final double wheelRadiusMeters = 0.0508;
    return new SwerveModuleState(
        inputs.driveVelocityRadPerSec * wheelRadiusMeters,
        new Rotation2d(
            inputs.turnPositionsRad.length > 0
                ? inputs.turnPositionsRad[inputs.turnPositionsRad.length - 1]
                : 0.0));
  }

  /**
   * Routes target voltage demands safely down into the IO execution layer.
   *
   * @param volts Target requested feedforward / PID voltage calculated securely.
   */
  public void setDriveVoltage(double volts) {
    io.setDriveVoltage(volts);
  }

  /**
   * Routes steer voltage demands safely down into the IO execution layer.
   *
   * @param volts Target requested feedforward / PID voltage calculated securely.
   */
  public void setTurnVoltage(double volts) {
    io.setTurnVoltage(volts);
  }

  /**
   * Commands strict stator current limitations mapped directly proportional against Battery Drop
   * (Load Shedding).
   *
   * @param amps Absolute ceiling limit mapped securely via the {@link
   *     com.marslib.power.MARSPowerManager}.
   */
  public void setCurrentLimit(double amps) {
    io.setCurrentLimit(amps);
  }
}
