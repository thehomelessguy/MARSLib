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
import com.marslib.faults.MARSFaultManager;
import com.marslib.util.LoggedTunableNumber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hardware binding layer for singular Rotary Mechanisms utilizing CTRE TalonFX (Kraken/Falcon)
 * motors.
 *
 * <p>Students: This IO class natively integrates CTRE's Phoenix 6 API. Notice how we use
 * `StatusSignal` to fetch velocity and positions. Phoenix 6 runs Odometry internally at 1000Hz
 * (1kHz) natively. When calling {@link #setClosedLoopPosition}, this layer leverages Phoenix 6
 * "Motion Magic", which is a trapezoidal profile generator running physically on the motor
 * controller directly.
 */
public class RotaryMechanismIOTalonFX implements RotaryMechanismIO {

  private final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Double> closedLoopReferenceSlope;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  private final double gearRatio;

  /** Tracks the last CAN-applied stator current limit to avoid redundant writes. */
  private double lastAppliedCurrentLimit = 40.0;

  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  /**
   * Initializes the TalonFX hardware and pre-configures MotionMagic.
   *
   * @param motorId The physical CAN ID of the TalonFX.
   * @param canbus The CANBus string (e.g. "rio" or "canivore").
   * @param gearRatio Total mechanical gear reduction from the motor to the output shaft (e.g. 10.0
   *     for 10:1).
   * @param inverted Whether the positive motor rotation direction drives the mechanism "forward".
   */
  public RotaryMechanismIOTalonFX(int motorId, String canbus, double gearRatio, boolean inverted) {
    this.gearRatio = gearRatio;
    this.motor = new TalonFX(motorId, canbus);

    kP = new LoggedTunableNumber("RotaryMechanism_" + motorId + "/kP", 2.0);
    kI = new LoggedTunableNumber("RotaryMechanism_" + motorId + "/kI", 0.0);
    kD = new LoggedTunableNumber("RotaryMechanism_" + motorId + "/kD", 0.0);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted =
        inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // Motion Magic Default Bounds
    config.MotionMagic.MotionMagicCruiseVelocity = 100.0; // rotations per second (motor domain)
    config.MotionMagic.MotionMagicAcceleration = 200.0;
    config.MotionMagic.MotionMagicJerk = 0.0; // Instant

    // CTRE PID for Motion Magic Profile Tracking
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
  }

  @Override
  public void updateInputs(RotaryMechanismIOInputs inputs) {
    boolean ok =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, statorCurrent, closedLoopReferenceSlope)
            .isOK();

    inputs.hasHardwareConnected = ok;
    if (!ok) {
      MARSFaultManager.reportHardwareDisconnect("RotaryMechanism_" + motor.getDeviceID());
    }

    // Convert motor rotations -> mechanism rads
    double radsPerMotorRotation = (2 * Math.PI) / gearRatio;

    inputs.positionRad = position.getValueAsDouble() * radsPerMotorRotation;
    inputs.velocityRadPerSec = velocity.getValueAsDouble() * radsPerMotorRotation;
    inputs.targetVelocityRadPerSec =
        closedLoopReferenceSlope.getValueAsDouble() * radsPerMotorRotation;
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
   * <p>Students: The target parameter `positionRad` is passed as pure mechanism Radians. This class
   * handles scaling that value up through the Gear Ratio automatically so the physical TalonFX
   * understands how many raw Motor Rotations it needs to accomplish to hit your target.
   *
   * @param positionRad The target angle of the total mechanism (radians).
   * @param feedforwardVolts Dynamic SysId feedforward volts aiding the onboard PID.
   */
  @Override
  public void setClosedLoopPosition(double positionRad, double feedforwardVolts) {
    // Convert target mechanism bounds natively into Motor domain internal rotations target for
    // MotionMagic
    double motorTargetRotations = positionRad / ((2 * Math.PI) / gearRatio);
    motor.setControl(
        motionMagicRequest.withPosition(motorTargetRotations).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setEncoderPosition(double positionRad) {
    double motorTargetRotations = positionRad / ((2 * Math.PI) / gearRatio);
    motor.setPosition(motorTargetRotations);
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
