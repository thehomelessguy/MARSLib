package com.marslib.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SwerveModuleIOSim implements SwerveModuleIO {

  // Roughly standard L1 gearing and generic MOI
  private final DCMotorSim driveSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.025, 6.75),
          DCMotor.getKrakenX60(1));

  private final DCMotorSim turnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, 21.4),
          DCMotor.getKrakenX60(1));

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public SwerveModuleIOSim() {}

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    driveSim.update(0.02);
    turnSim.update(0.02);

    inputs.hasHardwareConnected = true;

    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();

    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.turnAppliedVolts = turnAppliedVolts;

    inputs.driveCurrentAmps = driveSim.getCurrentDrawAmps();
    inputs.turnCurrentAmps = turnSim.getCurrentDrawAmps();

    // Feed arrays containing precisely the standard 50hz tick for simulation parity
    inputs.drivePositionsRad = new double[] {driveSim.getAngularPositionRad()};
    inputs.turnPositionsRad = new double[] {turnSim.getAngularPositionRad()};
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = volts;
    driveSim.setInputVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = volts;
    turnSim.setInputVoltage(volts);
  }

  @Override
  public void setCurrentLimit(double amps) {
    // Simulation does not strictly enforce brownout limits realistically on the objects yet
  }
}
