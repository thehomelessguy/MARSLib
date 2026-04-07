package com.marslib.mechanisms;

import static edu.wpi.first.units.Units.Volts;

import com.marslib.util.LoggedTunableNumber;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private final LoggedTunableNumber kS = new LoggedTunableNumber("Shooter/kS", 0.0);
  private final LoggedTunableNumber kV = new LoggedTunableNumber("Shooter/kV", 0.0);
  private final LoggedTunableNumber kA = new LoggedTunableNumber("Shooter/kA", 0.0);

  private SimpleMotorFeedforward feedforward;
  private final SysIdRoutine sysIdRoutine;

  /**
   * Constructs the shooter subsystem.
   *
   * @param io The hardware abstraction layer for the shooter flywheel motor.
   */
  public MARSShooter(FlywheelIO io) {
    this.io = io;
    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());

    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (edu.wpi.first.units.measure.Voltage volts) -> {
                  io.setVoltage(volts.in(Volts));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    int id = this.hashCode();
    boolean sChanged = kS.hasChanged(id);
    boolean vChanged = kV.hasChanged(id);
    boolean aChanged = kA.hasChanged(id);

    if (sChanged || vChanged || aChanged) {
      feedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
    }
  }

  /**
   * Returns a command that continuously spins the flywheel at the default scoring velocity.
   *
   * @return A {@link Command} that runs the flywheel at ~400 rad/s using closed-loop control.
   */
  public Command spinUpCommand() {
    return this.run(() -> setClosedLoopVelocity(400.0));
  }

  /**
   * Directly sets the shooter motor voltage. Used for open-loop manual control.
   *
   * @param volts The voltage to apply.
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  private double targetVelocityRadPerSec = 0.0;

  /**
   * Runs the flywheel at a target velocity using the motor's internal closed-loop controller and
   * dynamically injects the calculated feedforward voltage.
   *
   * @param speed Target velocity in radians per second.
   */
  public void setClosedLoopVelocity(double speed) {
    this.targetVelocityRadPerSec = speed;
    double ffVolts = feedforward.calculate(speed);
    io.setClosedLoopVelocity(speed, ffVolts);
  }

  public boolean isAtTolerance() {
    return Math.abs(inputs.velocityRadPerSec - targetVelocityRadPerSec) < 20.0;
  }

  /**
   * Generates a SysId Quasistatic characterization command.
   *
   * @param direction The direction of the quasistatic routine (Forward/Reverse).
   * @return The SysId Command to execute.
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Generates a SysId Dynamic characterization command.
   *
   * @param direction The direction of the dynamic routine (Forward/Reverse).
   * @return The SysId Command to execute.
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
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
