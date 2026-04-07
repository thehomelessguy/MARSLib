package com.marslib.swerve;

import com.marslib.simulation.SwerveChassisPhysics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.SwerveConstants;

/**
 * Simulation IO layer for a single swerve module.
 *
 * <p>The <b>drive</b> motor physics are handled entirely by the centralized {@link
 * SwerveChassisPhysics} engine (which models wheel slip, friction, and battery sag). This class
 * reads wheel angular velocity back from that engine via {@link
 * SwerveChassisPhysics#getWheelOmegaRadPerSec(int)} to ensure a single source of truth for drive
 * dynamics.
 *
 * <p>The <b>turn</b> motor remains a local {@link DCMotorSim} since steering dynamics are
 * independent of chassis-level traction physics.
 */
public class SwerveModuleIOSim implements SwerveModuleIO {
  private final int moduleIndex;

  /** Reference to the centralized chassis physics engine. May be null before physics init. */
  private SwerveChassisPhysics chassisPhysics;

  /** Local sim for the turn motor (steering is independent of chassis traction physics). */
  private final DCMotorSim turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60Foc(1), 0.004, SwerveConstants.TURN_GEAR_RATIO),
          DCMotor.getKrakenX60Foc(1));

  /** Accumulated drive position from the physics engine (radians at the wheel output shaft). */
  private double drivePositionRad = 0.0;

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  /**
   * Constructs a simulated swerve module IO.
   *
   * @param moduleIndex The module index (0=FL, 1=FR, 2=BL, 3=BR), used to query the correct wheel
   *     omega from the physics engine.
   */
  public SwerveModuleIOSim(int moduleIndex) {
    this.moduleIndex = moduleIndex;
  }

  /**
   * Injects the centralized chassis physics reference after it has been constructed.
   *
   * <p>This setter exists because the physics engine and the IO layers have a circular
   * initialization dependency: SwerveDrive creates modules first, then the physics engine.
   *
   * @param physics The {@link SwerveChassisPhysics} instance to read wheel omegas from.
   */
  public void setChassisPhysics(SwerveChassisPhysics physics) {
    this.chassisPhysics = physics;
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    turnSim.update(Constants.LOOP_PERIOD_SECS);

    inputs.hasHardwareConnected = true;

    // Drive state comes from the centralized physics engine (single source of truth)
    double wheelOmega = 0.0;
    if (chassisPhysics != null) {
      wheelOmega = chassisPhysics.getWheelOmegaRadPerSec(moduleIndex);
    }
    drivePositionRad += wheelOmega * Constants.LOOP_PERIOD_SECS;

    inputs.driveVelocityRadPerSec = wheelOmega;
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();

    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.turnAppliedVolts = turnAppliedVolts;

    inputs.driveCurrentAmps = 0.0; // Current is tracked globally in SwerveChassisPhysics
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();

    inputs.drivePositionsRad = new double[] {drivePositionRad};
    inputs.turnPositionsRad = new double[] {turnSim.getAngularPositionRad()};
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = volts;
    // Drive voltage is consumed by SwerveChassisPhysics, not a local DCMotorSim
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = volts;
    turnSim.setInputVoltage(volts);
  }

  private double lastCurrentLimitAmps = 0.0;

  @Override
  public void setCurrentLimit(double amps) {
    // Current limiting is enforced at the physics engine level via battery voltage clamping
    lastCurrentLimitAmps = amps;
  }

  public double getCurrentLimitAmps() {
    return lastCurrentLimitAmps;
  }
}
