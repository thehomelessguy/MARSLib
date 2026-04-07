package com.marslib.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.marslib.util.LoggedTunableNumber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hardware binding layer for singular Linear Mechanisms (Elevators, Extensors) utilizing CTRE
 * TalonFX (Kraken/Falcon) motors.
 *
 * <p>Students: This IO class natively integrates CTRE's Phoenix 6 API identically to the Rotary
 * layer, except it manages the `spoolDiameterMeters` allowing it to physically convert Motor
 * Rotations into absolute Metric Extents (meters).
 *
 * <p>Phoenix 6 runs current limits natively on the stator coil at 1000Hz (1kHz) providing robust
 * physical limits. When calling {@link #setClosedLoopPosition}, this layer leverages Phoenix 6
 * "Motion Magic", which is a trapezoidal profile generator running physically on the motor
 * controller directly.
 */
public class LinearMechanismIOTalonFX implements LinearMechanismIO {

  private final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Double> closedLoopReferenceSlope;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  private final double gearRatio;
  private final double spoolDiameterMeters;

  /** Tracks the last CAN-applied stator current limit to avoid redundant writes. */
  private double lastAppliedCurrentLimit = 40.0;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  /**
   * Initializes the TalonFX hardware and pre-configures Linear MotionMagic parameters.
   *
   * @param motorId The physical CAN ID of the TalonFX.
   * @param canbus The CANBus string (e.g. "rio" or "canivore").
   * @param gearRatio Total mechanical gear reduction from the motor to the spool.
   * @param spoolDiameterMeters The physical diameter of the output spool pulling the belt/string.
   * @param inverted Whether positive rotation extends the mechanism outward.
   */
  private final TalonFX[] followers;

  public LinearMechanismIOTalonFX(
      int motorId, String canbus, double gearRatio, double spoolDiameterMeters, boolean inverted) {
    this(motorId, new int[0], new boolean[0], canbus, gearRatio, spoolDiameterMeters, inverted);
  }

  public LinearMechanismIOTalonFX(
      int leaderId,
      int[] followerIds,
      boolean[] opposeLeader,
      String canbus,
      double gearRatio,
      double spoolDiameterMeters,
      boolean inverted) {
    this.gearRatio = gearRatio;
    this.spoolDiameterMeters = spoolDiameterMeters;
    this.motor = new TalonFX(leaderId, canbus);

    kP = new LoggedTunableNumber("LinearMechanism_" + leaderId + "/kP", 2.0);
    kI = new LoggedTunableNumber("LinearMechanism_" + leaderId + "/kI", 0.0);
    kD = new LoggedTunableNumber("LinearMechanism_" + leaderId + "/kD", 0.0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity = 200.0;
    config.MotionMagic.MotionMagicAcceleration = 400.0;

    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    closedLoopReferenceSlope = motor.getClosedLoopReferenceSlope();

    position.setUpdateFrequency(50.0);
    velocity.setUpdateFrequency(50.0);
    appliedVolts.setUpdateFrequency(50.0);
    statorCurrent.setUpdateFrequency(50.0);
    closedLoopReferenceSlope.setUpdateFrequency(50.0);

    motor.optimizeBusUtilization();

    followers = new TalonFX[followerIds.length];
    for (int i = 0; i < followerIds.length; i++) {
      followers[i] = new TalonFX(followerIds[i], canbus);
      followers[i].getConfigurator().apply(config);
      followers[i].setControl(new com.ctre.phoenix6.controls.Follower(leaderId, opposeLeader[i]));
      followers[i].optimizeBusUtilization();
    }
  }

  @Override
  public void updateInputs(LinearMechanismIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        position, velocity, appliedVolts, statorCurrent, closedLoopReferenceSlope);

    inputs.hasHardwareConnected = true;

    double metersPerMotorRotation = (spoolDiameterMeters * Math.PI) / gearRatio;

    inputs.positionMeters = position.getValueAsDouble() * metersPerMotorRotation;
    inputs.velocityMetersPerSec = velocity.getValueAsDouble() * metersPerMotorRotation;
    inputs.targetVelocityMetersPerSec =
        closedLoopReferenceSlope.getValueAsDouble() * metersPerMotorRotation;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = new double[] {statorCurrent.getValueAsDouble()};

    // Live Auto-Tuning Check
    boolean pChanged = kP.hasChanged(motor.getDeviceID());
    boolean iChanged = kI.hasChanged(motor.getDeviceID());
    boolean dChanged = kD.hasChanged(motor.getDeviceID());

    if (pChanged || iChanged || dChanged) {
      com.ctre.phoenix6.configs.Slot0Configs slot0 = new com.ctre.phoenix6.configs.Slot0Configs();
      motor.getConfigurator().refresh(slot0);
      slot0.kP = kP.get();
      slot0.kI = kI.get();
      slot0.kD = kD.get();
      motor.getConfigurator().apply(slot0);
    }
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * Pushes a target state using Phoenix 6 Motion Magic.
   *
   * <p>Students: The target parameter `positionMeters` is passed natively as Extent Meters. This
   * class handles scaling that value up through the Spool Circumference and Gear Ratio
   * automatically so the TalonFX understands how many internal motor rotations it must make to
   * drive the elevator to your target height.
   *
   * @param positionMeters The target extension length of the mechanism (meters).
   * @param feedforwardVolts Dynamic SysId feedforward volts (usually Gravity compensation) aiding
   *     the onboard PID.
   */
  @Override
  public void setClosedLoopPosition(double positionMeters, double feedforwardVolts) {
    double motorTargetRotations = positionMeters / ((spoolDiameterMeters * Math.PI) / gearRatio);
    motor.setControl(
        motionMagicRequest.withPosition(motorTargetRotations).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setCurrentLimit(double amps) {
    if (Math.abs(amps - lastAppliedCurrentLimit) < 1.0) {
      return;
    }
    lastAppliedCurrentLimit = amps;
    CurrentLimitsConfigs limit = new CurrentLimitsConfigs();
    motor.getConfigurator().refresh(limit);
    limit.StatorCurrentLimit = amps;
    motor.getConfigurator().apply(limit);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    motor.getConfigurator().refresh(config);
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }
}
