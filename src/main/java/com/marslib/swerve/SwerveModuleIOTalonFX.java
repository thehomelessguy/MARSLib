package com.marslib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Hardware IO implementation for a Swerve Module using CTRE TalonFX motors and a CANcoder. */
public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX turnMotor;

  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<AngularVelocity> turnVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Voltage> turnAppliedVolts;
  private final StatusSignal<Current> driveCurrent;
  private final StatusSignal<Current> turnCurrent;

  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut turnVoltageRequest = new VoltageOut(0.0);

  private final int odometryId;

  public SwerveModuleIOTalonFX(int driveMotorId, int turnMotorId, String canbus) {
    driveMotor = new TalonFX(driveMotorId, canbus);
    turnMotor = new TalonFX(turnMotorId, canbus);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit =
        frc.robot.SwerveConstants.DRIVE_STATOR_CURRENT_LIMIT;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveConfig);

    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnConfig.CurrentLimits.StatorCurrentLimit =
        frc.robot.SwerveConstants.TURN_STATOR_CURRENT_LIMIT;
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnMotor.getConfigurator().apply(turnConfig);

    driveVelocity = driveMotor.getVelocity();
    turnVelocity = turnMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    turnAppliedVolts = turnMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();
    turnCurrent = turnMotor.getStatorCurrent();

    // Ensure these signals run at standard config frequency
    double updateHz = frc.robot.Constants.DriveConstants.TELEMETRY_HZ;
    driveVelocity.setUpdateFrequency(updateHz);
    turnVelocity.setUpdateFrequency(updateHz);
    driveAppliedVolts.setUpdateFrequency(updateHz);
    turnAppliedVolts.setUpdateFrequency(updateHz);
    driveCurrent.setUpdateFrequency(updateHz);
    turnCurrent.setUpdateFrequency(updateHz);

    // Register position signals to Odometry thread
    odometryId =
        PhoenixOdometryThread.getInstance()
            .registerModule(driveMotor.getPosition(), turnMotor.getPosition());

    driveMotor.optimizeBusUtilization();
    turnMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    // Query the bulk 50hz telemetry
    BaseStatusSignal.refreshAll(
        driveVelocity, turnVelocity,
        driveAppliedVolts, turnAppliedVolts,
        driveCurrent, turnCurrent);

    inputs.hasHardwareConnected = true; // Assume true if no error during refresh mapping
    // Convert from motor-domain (rotations) to output-shaft-domain (radians at the wheel)
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble())
            / frc.robot.SwerveConstants.DRIVE_GEAR_RATIO;
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble())
            / frc.robot.SwerveConstants.TURN_GEAR_RATIO;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();

    // Drain the high-frequency buffer
    PhoenixOdometryThread.SyncData data =
        PhoenixOdometryThread.getInstance().getSyncData(odometryId);

    // Convert to radians
    inputs.drivePositionsRad = new double[data.drivePositions.length];
    inputs.turnPositionsRad = new double[data.turnPositions.length];
    inputs.odometryTimestamps = new double[data.timestamps.length];

    for (int i = 0; i < data.drivePositions.length; i++) {
      // Convert from motor rotations to output-shaft radians (post-gearing)
      inputs.drivePositionsRad[i] =
          Units.rotationsToRadians(data.drivePositions[i])
              / frc.robot.SwerveConstants.DRIVE_GEAR_RATIO;
      inputs.turnPositionsRad[i] =
          Units.rotationsToRadians(data.turnPositions[i])
              / frc.robot.SwerveConstants.TURN_GEAR_RATIO;
      inputs.odometryTimestamps[i] = data.timestamps[i];
    }
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnMotor.setControl(turnVoltageRequest.withOutput(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    driveMotor.getConfigurator().refresh(config);
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveMotor.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    turnMotor.getConfigurator().refresh(config);
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnMotor.getConfigurator().apply(config);
  }

  @Override
  public void setCurrentLimit(double amps) {
    CurrentLimitsConfigs limits = new CurrentLimitsConfigs();
    driveMotor.getConfigurator().refresh(limits);
    limits.StatorCurrentLimit = amps;
    driveMotor.getConfigurator().apply(limits);
  }
}
