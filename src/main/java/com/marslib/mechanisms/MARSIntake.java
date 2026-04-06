package com.marslib.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MARSIntake extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public MARSIntake(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command runIntakeCommand() {
    // Basic typical intake speed (e.g. 12 volts)
    return this.startEnd(() -> io.setVoltage(12.0), () -> io.setVoltage(0.0));
  }

  public Command runReverseCommand() {
    return this.startEnd(() -> io.setVoltage(-12.0), () -> io.setVoltage(0.0));
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stop() {
    io.setVoltage(0.0);
  }
}
