package com.marslib.mechanisms;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem controlling the robot's shooter flywheel mechanism.
 *
 * <p>The shooter uses a velocity-controlled flywheel to launch game pieces (Fuel) at the target
 * Hub. It accepts a {@link FlywheelIO} implementation via dependency injection, allowing seamless
 * switching between real hardware ({@link FlywheelIOTalonFX}) and physics simulation ({@link
 * FlywheelIOSim}).
 *
 * <p>Students: Use {@link #spinUpCommand()} to rev the flywheel to scoring speed. The {@link
 * MARSSuperstructure} automatically manages spin-up timing relative to the elevator and arm state.
 */
public class MARSShooter extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  /**
   * Constructs the shooter subsystem.
   *
   * @param io The hardware abstraction layer for the shooter flywheel motor.
   */
  public MARSShooter(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  /**
   * Returns a command that continuously spins the flywheel at the default scoring velocity.
   *
   * @return A {@link Command} that runs the flywheel at ~400 rad/s using closed-loop control.
   */
  public Command spinUpCommand() {
    return this.run(() -> io.setClosedLoopVelocity(400.0, 0.0));
  }

  /**
   * Directly sets the shooter motor voltage. Used for open-loop manual control.
   *
   * @param volts The voltage to apply.
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Runs the flywheel at a target velocity using the motor's internal closed-loop controller.
   *
   * @param speed Target velocity in radians per second.
   * @param ff Feedforward voltage to add to the closed-loop output.
   */
  public void setClosedLoopVelocity(double speed, double ff) {
    io.setClosedLoopVelocity(speed, ff);
  }

  /**
   * Gets the current velocity of the shooter flywheel.
   *
   * @return Velocity in radians per second.
   */
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }
}
