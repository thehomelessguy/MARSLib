package com.marslib.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class MARSShooter extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public MARSShooter(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command spinUpCommand() {
    // E.g. ~400 rad/s
    return this.run(() -> io.setClosedLoopVelocity(400.0, 0.0));
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setClosedLoopVelocity(double speed, double ff) {
    io.setClosedLoopVelocity(speed, ff);
  }
}
