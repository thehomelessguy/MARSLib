package com.marslib.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling the robot's intake roller mechanism.
 *
 * <p>The intake uses a single flywheel motor to pull game pieces into the robot. It accepts a
 * {@link FlywheelIO} implementation via dependency injection, allowing seamless switching between
 * real hardware ({@link FlywheelIOTalonFX}) and physics simulation ({@link FlywheelIOSim}).
 *
 * <p>Students: Bind {@link #runIntakeCommand()} and {@link #runReverseCommand()} to controller
 * buttons in {@code RobotContainer} for basic intake/outtake control.
 */
public class MARSIntake extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /**
   * Constructs the intake subsystem.
   *
   * @param io The hardware abstraction layer for the intake roller motor.
   */
  public MARSIntake(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /**
   * Returns a command that runs the intake forward at full voltage, stopping when cancelled.
   *
   * @return A {@link Command} that spins the roller inward at 12V.
   */
  public Command runIntakeCommand() {
    return this.startEnd(() -> io.setVoltage(12.0), () -> io.setVoltage(0.0));
  }

  /**
   * Returns a command that runs the intake in reverse at full voltage, stopping when cancelled.
   *
   * @return A {@link Command} that spins the roller outward at -12V for ejecting jammed pieces.
   */
  public Command runReverseCommand() {
    return this.startEnd(() -> io.setVoltage(-12.0), () -> io.setVoltage(0.0));
  }

  /**
   * Directly sets the intake motor voltage. Intended for use by the {@link MARSSuperstructure}.
   *
   * @param volts The voltage to apply (positive = inward, negative = outward).
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /** Stops the intake motor immediately. */
  public void stop() {
    io.setVoltage(0.0);
  }
}
