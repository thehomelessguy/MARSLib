package com.marslib.swerve;

import com.marslib.simulation.SwerveChassisPhysics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveConstants;
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

  private double lastDriveVoltage = 0.0;
  private final SwerveModulePosition[] cachedDeltas = new SwerveModulePosition[20];
  private int cachedDeltaCount = 0;

  /**
   * Constructs a generic Swerve Module boundary structure.
   *
   * @param index The ID/Index (0=FL, 1=FR, 2=BL, 3=BR) for structural AdvantageKit logging keys.
   * @param io The implementation-specific IO layer (TalonFX or Sim).
   */
  public SwerveModule(int index, SwerveModuleIO io) {
    this.index = index;
    this.io = io;
    for (int i = 0; i < cachedDeltas.length; i++) {
      cachedDeltas[i] = new SwerveModulePosition(0.0, Rotation2d.fromRadians(0.0));
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("SwerveDrive/Module" + index, inputs);

    // Update statically allocated positional cache
    cachedDeltaCount = Math.min(inputs.drivePositionsRad.length, cachedDeltas.length);
    for (int i = 0; i < cachedDeltaCount; i++) {
      cachedDeltas[i].distanceMeters =
          inputs.drivePositionsRad[i] * SwerveConstants.WHEEL_RADIUS_METERS;
      cachedDeltas[i].angle = Rotation2d.fromRadians(inputs.turnPositionsRad[i]);
    }
  }

  /** Returns the count of valid samples populated in the buffer during the last periodic cycle. */
  public int getDeltaCount() {
    return cachedDeltaCount;
  }

  /** Accesses the pre-allocated cache representing positional samples without creating arrays. */
  public SwerveModulePosition getCachedDelta(int i) {
    if (cachedDeltaCount == 0) return cachedDeltas[0];
    return cachedDeltas[i];
  }

  /**
   * Returns the hardware timestamps of the high-frequency positional records.
   *
   * @return Array of FPGA/Hardware timestamps tightly synced with {@link #getPositionDeltas()}.
   */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /**
   * Returns the most recent physical location recorded by the drive motors.
   *
   * @return A singular {@link SwerveModulePosition} bounding distance traveled and heading.
   */
  public SwerveModulePosition getLatestPosition() {
    if (cachedDeltaCount == 0) return cachedDeltas[0];
    return cachedDeltas[cachedDeltaCount - 1];
  }

  /**
   * Translates drive wheel RPS and turn module radians into WPILib Velocity metrics.
   *
   * @return A {@link SwerveModuleState} tracking linear velocity (m/s) and angular heading.
   */
  public SwerveModuleState getLatestState() {
    return new SwerveModuleState(
        inputs.driveVelocityRadPerSec * SwerveConstants.WHEEL_RADIUS_METERS,
        Rotation2d.fromRadians(
            inputs.turnPositionsRad.length > 0
                ? inputs.turnPositionsRad[inputs.turnPositionsRad.length - 1]
                : 0.0));
  }

  /**
   * Optimizes and applies a desired module state (drive speed + turn angle) with closed-loop turn
   * control and simple voltage feedforward for driving.
   *
   * <p>Students: This method first calls {@code SwerveModuleState.optimize()} to minimize turn
   * motor rotation. It then applies a proportional voltage controller to steer the module and a
   * linear voltage mapping for drive speed.
   *
   * @param desiredState The target speed and angle for this module.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Get current module angle
    Rotation2d currentAngle =
        Rotation2d.fromRadians(
            inputs.turnPositionsRad.length > 0
                ? inputs.turnPositionsRad[inputs.turnPositionsRad.length - 1]
                : 0.0);

    // Optimize to minimize turn rotation (may flip drive direction)
    desiredState.optimize(currentAngle);

    // Turn voltage: proportional controller on angular error
    double angleErrorRad = desiredState.angle.minus(currentAngle).getRadians();

    // Cosine Compensation: Scale drive speed by cosine of error angle
    // This prevents the robot from driving while the wheels are sideways, eliminating drift.
    double driveVoltage =
        (desiredState.speedMetersPerSecond * Math.cos(angleErrorRad))
            * SwerveConstants.NOMINAL_BATTERY_VOLTAGE
            / SwerveConstants.MAX_LINEAR_SPEED_MPS;

    double turnVoltage = angleErrorRad * SwerveConstants.TURN_KP;
    turnVoltage =
        Math.max(
            -SwerveConstants.NOMINAL_BATTERY_VOLTAGE,
            Math.min(SwerveConstants.NOMINAL_BATTERY_VOLTAGE, turnVoltage));

    lastDriveVoltage = driveVoltage;
    io.setDriveVoltage(driveVoltage);
    io.setTurnVoltage(turnVoltage);
  }

  /**
   * Routes target voltage demands safely down into the IO execution layer.
   *
   * @param volts Target requested feedforward / PID voltage calculated securely.
   */
  public void setDriveVoltage(double volts) {
    lastDriveVoltage = volts;
    io.setDriveVoltage(volts);
  }

  /** Used strictly for extracting injected sim forces. */
  public double getSimDriveVoltage() {
    return lastDriveVoltage;
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

  /**
   * Injects the centralized chassis physics reference into the underlying IO layer if it is a
   * {@link SwerveModuleIOSim}. This ensures the sim IO reads wheel omegas from the single physics
   * engine rather than running its own duplicate motor simulation.
   *
   * <p>No-op if the IO layer is not a sim implementation.
   *
   * @param physics The {@link SwerveChassisPhysics} instance to inject.
   */
  public void injectChassisPhysics(SwerveChassisPhysics physics) {
    if (io instanceof SwerveModuleIOSim) {
      ((SwerveModuleIOSim) io).setChassisPhysics(physics);
    }
  }
}
