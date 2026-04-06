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

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX motor;

  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> velocitySignal;
  private final StatusSignal<edu.wpi.first.units.measure.Voltage> voltageSignal;
  private final StatusSignal<edu.wpi.first.units.measure.Current> statorCurrentSignal;

  private final VoltageOut voltageRequest = new VoltageOut(0.0).withUpdateFreqHz(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withUpdateFreqHz(0);

  private double targetVelocityRadPerSec = 0.0;

  public FlywheelIOTalonFX(int motorId, String canBus, boolean invert) {
    motor = new TalonFX(motorId, canBus);

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
