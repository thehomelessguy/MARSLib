package com.marslib.mechanisms;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Real hardware implementation of {@link FlywheelIO} using a CTRE TalonFX motor controller.
 *
 * <p>Supports open-loop voltage control and closed-loop velocity control via Phoenix 6 onboard PID.
 * Used for shooter and intake flywheels on the physical robot.
 */
public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX motor;

  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> velocitySignal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> voltageSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> statorCurrentSignal;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withUpdateFreqHz(0);

  private double targetVelocityRadPerSec = 0.0;
  private double lastAppliedCurrentLimit = 40.0;

  private final TalonFX[] followers;

  /**
   * Constructs a single Flywheel mechanism using a direct CTRE TalonFX.
   *
   * @param motorId The physical CAN ID of the primary TalonFX.
   * @param canBus The CANBus string (e.g. "rio" or "canivore").
   * @param invert Whether to invert the motor output.
   */
  public FlywheelIOTalonFX(int motorId, String canBus, boolean invert) {
    this(motorId, new int[0], new boolean[0], canBus, invert);
  }

  /**
   * Constructs a master-follower Flywheel configuration (e.g., dual-motor shooter).
   *
   * @param leaderId The physical CAN ID of the master TalonFX.
   * @param followerIds Array of CAN IDs for the follower motors.
   * @param opposeLeader Array of booleans denoting if the follower should spin opposite to the
   *     leader.
   * @param canBus The CANBus string.
   * @param invert Whether the leader is inverted.
   */
  public FlywheelIOTalonFX(
      int leaderId, int[] followerIds, boolean[] opposeLeader, String canBus, boolean invert) {
    motor = new TalonFX(leaderId, canBus);

    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // defaults coast for shooters/intakes

    // Default basic PID
    config.Slot0.kP = 0.1;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;

    motor.getConfigurator().apply(config);

    velocitySignal = motor.getVelocity();
    voltageSignal = motor.getMotorVoltage();
    statorCurrentSignal = motor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocitySignal, voltageSignal, statorCurrentSignal);
    motor.optimizeBusUtilization();

    followers = new TalonFX[followerIds.length];
    for (int i = 0; i < followerIds.length; i++) {
      followers[i] = new TalonFX(followerIds[i], canBus);
      followers[i].getConfigurator().apply(config);
      followers[i].setControl(new com.ctre.phoenix6.controls.Follower(leaderId, opposeLeader[i]));
      followers[i].optimizeBusUtilization();
    }
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.hasHardwareConnected =
        BaseStatusSignal.refreshAll(velocitySignal, voltageSignal, statorCurrentSignal).isOK();

    // CTRE returns rotations, convert to radians
    inputs.velocityRadPerSec = velocitySignal.getValueAsDouble() * Math.PI * 2.0;
    inputs.targetVelocityRadPerSec = targetVelocityRadPerSec;
    inputs.appliedVolts = voltageSignal.getValueAsDouble();
    inputs.currentAmps = new double[] {statorCurrentSignal.getValueAsDouble()};
  }

  @Override
  public void setVoltage(double volts) {
    targetVelocityRadPerSec = 0.0;
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setClosedLoopVelocity(double velocityRadPerSec, double feedforwardVolts) {
    targetVelocityRadPerSec = velocityRadPerSec;
    // CTRE velocity is in rotations per second
    double rps = velocityRadPerSec / (Math.PI * 2.0);
    motor.setControl(velocityRequest.withVelocity(rps).withFeedForward(feedforwardVolts));
  }

  @Override
  public void setCurrentLimit(double amps) {
    if (Math.abs(amps - lastAppliedCurrentLimit) < 1.0) {
      return;
    }
    lastAppliedCurrentLimit = amps;
    CurrentLimitsConfigs config = new CurrentLimitsConfigs();
    motor.getConfigurator().refresh(config);
    config.StatorCurrentLimit = amps;
    config.StatorCurrentLimitEnable = true;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    motor.getConfigurator().refresh(config);
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motor.getConfigurator().apply(config);
  }
}
